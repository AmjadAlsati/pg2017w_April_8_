/**
 * Defines helper function for seding and receiving 11p packet, independently
 * from the platform (i.e., Cohda or UNEX)
 */

#include "cooperative_driving/networking/transceiver.h"

#include <errno.h>
#include <linux/wireless.h>
#include <math.h>
#include <sys/ioctl.h>
#include "cooperative_driving/networking/cohda_defs.h"
#include "cooperative_driving/networking/ieee80211.h"
#include "cooperative_driving/networking/ieee80211_radiotap.h"
#include "cooperative_driving/networking/radiotap.h"

#include <stdio.h>

void dump_buffer(char *buf, int len)
{
  int i;
  for (i = 0; i < len; ++i)
  {
    char c = buf[i];
    if (c >= '0')
    {
      printf("%c", buf[i]);
    }
    else
    {
      printf("0x%02X ", buf[i]);
    }
  }
  printf("\n");
}

void print_mac(unsigned char mac[ETH_ALEN])
{
  printf("%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// send and receive functions for unex platform

/**
 * Set the channel for the device
 */
int set_channel(int fd, const char ifname[IFNAMSIZ], int channel);
/**
 * Set modulation and coding scheme for the device
 */
int set_mcs(int fd, const char ifname[IFNAMSIZ], enum MK2MCS mcs);
/**
 * Set tx power for the device
 */
int set_txpower(const char ifname[IFNAMSIZ], tMK2Power txpower);
/**
 * Get hw mac address
 */
int get_mac_address(int fd, const char ifname[IFNAMSIZ], unsigned char mac[6]);

device_t *open_device(tTxOpts *params)
{
  device_t *device = malloc(sizeof(device_t));
  tTx *pTx = &device->tx;
  device->txArgs = *params;
  tTxOpts *pTxOpts = &device->txArgs;

  char *device_name = params->pInterfaceName;

  pTx->Fd = -1;  // File description invalid
  pTx->IfIdx = -1;
  pTx->SeqNum = 0;
  pTx->logFile = 0;

  // Create an output Buffer (could have large persistent Buf in pTx for speed)
  pTx->pBuf = (unsigned char *)malloc(0xFFFFU);
  if (pTx->pBuf == NULL)
  {
    printf("Fail: malloc() errno %d\n", errno);
  }
  pTx->pRxBuf = (unsigned char *)malloc(0xFFFFU);
  if (pTx->pRxBuf == NULL)
  {
    printf("Fail: malloc() errno %d\n", errno);
  }

  // PreLoad the Ethernet Header (used in RAW frames)
  memcpy(pTx->EthHdr.h_source, pTxOpts->SrcAddr, ETH_ALEN);  // SA

  char *errbuf = malloc(sizeof(char) * PCAP_ERRBUF_SIZE);
  pTx->pcap = pcap_open_live(device_name, BUFSIZ, 1, 1000, errbuf);
  if (!pTx->pcap)
  {
    fprintf(stderr, "Couldn't open device %s: %s\n", device_name, errbuf);
    return 0;
  }

  pTx->Fd = socket(AF_INET, SOCK_DGRAM, 0);
  // set_channel(pTx->Fd, device_name, pTxOpts->TxCHOpts.ChannelNumber);
  // set_mcs(pTx->Fd, device_name, pTxOpts->TxCHOpts.pMCS);
  set_txpower(device_name, pTxOpts->TxCHOpts.TxPower);
  get_mac_address(pTx->Fd, device_name, pTxOpts->SrcAddr);

  free(errbuf);

  return device;
}

void close_device(device_t *device)
{
  close(device->tx.Fd);
  pcap_close(device->tx.pcap);
}

int send_frame(device_t *device, void *payload, int size, long txPower)
{
  // buffer where to store the data (radiotap, wifi header, payload, etc.)
  u8 u8aSendBuffer[2000];
  struct ieee80211_qos_hdr mac_hdr;

  // reset memory content
  memset(u8aSendBuffer, 0, sizeof(u8aSendBuffer));
  // make a pointer to the buffer
  u8 *pu8 = u8aSendBuffer;

  // insert radiotap header at the beginning, for nothing. it is ignored anyway
  memcpy(u8aSendBuffer, u8aRadiotapHeader, sizeof(u8aRadiotapHeader));
  // force the rate to use. the 5th is 12*2, which means 12Mbps in the 20MHz band
  // or 6Mbps in the 10MHz band
  pu8[OFFSET_RATE] = u8aRatesToUse[5];
  pu8[OFFSET_TXPOWER] = 20;
  pu8[OFFSET_ANTENNA] = 1;
  pu8 += sizeof(u8aRadiotapHeader);

  // set up source address
  memcpy(&mac_hdr, wifi_hdr, sizeof(wifi_hdr));
  memcpy(mac_hdr.addr2, device->txArgs.SrcAddr, 6);

  memcpy(pu8, &mac_hdr, sizeof(struct ieee80211_qos_hdr));
  pu8 += sizeof(struct ieee80211_qos_hdr);
  memcpy(pu8, llc_hdr, sizeof(llc_hdr));
  pu8 += sizeof(llc_hdr);

  // add payload
  memcpy(pu8, payload, size);
  pu8 += size;

  // set tx power just before sending
  set_txpower(device->txArgs.pInterfaceName, txPower);

  int r = pcap_inject(device->tx.pcap, u8aSendBuffer, pu8 - u8aSendBuffer);
  if (r != (pu8 - u8aSendBuffer))
  {
    perror("Trouble injecting packet");
    return 0;
  }

  return size;
}

int receive_frame(device_t *device, void *payload, int max_size, struct frame_info_t *frame_info, u8 tx_okay)
{
  struct pcap_pkthdr hdr;
  const unsigned char *packet;
  struct ieee80211_radiotap_iterator rti;
  struct ieee80211_hdr_3addr mac_hdr;
  u16 u16HeaderLen;
  u16 n80211HeaderLength;
  int bytes;
  PENUMBRA_RADIOTAP_DATA prd = { 0, 0, 0, 0, 0 };

  char errbuf[PCAP_ERRBUF_SIZE];

  // ensure that pcap_next call is blocking
  pcap_setnonblock(device->tx.pcap, 0, errbuf);
  // wait for next frame captured from the device
  if ((packet = pcap_next(device->tx.pcap, &hdr)) == 0)
  {
    return 0;
  }

  // get radiotap header length
  u16HeaderLen = (packet[2] + (packet[3] << 8));
  if (hdr.len < u16HeaderLen)
    return 0;

  // we still don't know whether frame is data or qos data. for now cast to data which is enough
  mac_hdr = *(struct ieee80211_hdr_3addr *)(packet + u16HeaderLen);
  memcpy(frame_info->src_address, ieee80211_get_SA((struct ieee80211_hdr *)&mac_hdr), sizeof(frame_info->src_address));
  // print_mac(frame_info->src_address);
  // printf("\n");
  // Comment in the following lines in order to filter by the last byte of the mac address
  //    if (frame_info->src_address[5] != '\x97')
  //        return 0;

  // check frame type
  if (ieee80211_is_data(mac_hdr.frame_control))
  {
    n80211HeaderLength = sizeof(struct ieee80211_hdr_3addr);
    if (ieee80211_is_data_qos(mac_hdr.frame_control))
    {
      n80211HeaderLength = sizeof(struct ieee80211_qos_hdr);
    }
  }
  else
  {
    //        printf("received something that is not a data frame\n");
    // we are only interested in data frames
    return 0;
  }

  // compute payload size
  bytes = hdr.len - (u16HeaderLen + n80211HeaderLength + 8);

  // init radiotap fields iterator
  int tmp = ieee80211_radiotap_iterator_init(&rti, (struct ieee80211_radiotap_header *)packet, hdr.len);
  //    if (ieee80211_radiotap_iterator_init(&rti, (struct ieee80211_radiotap_header *) packet, bytes) < 0)
  if (tmp < 0)
  {
    printf("%d\n", tmp);
    return 0;
  }

  // if the frame does not come with IEEE80211_RADIOTAP_RX_FLAGS, assume it was transmitted
  frame_info->was_tx = true;

  // loop through all radiotap fields available
  while ((ieee80211_radiotap_iterator_next(&rti)) == 0)
  {
    switch (rti.this_arg_index)
    {
      case IEEE80211_RADIOTAP_RATE:
        prd.m_nRate = (*rti.this_arg);
        frame_info->mcs = MK2MCS_DEFAULT;
        switch (prd.m_nRate)
        {
          case 2:  // 1mbps
            break;
          case 12:  // 6mbps
            frame_info->mcs = MK2MCS_R12BPSK;
            break;
          case 18:  // 9mbps
            frame_info->mcs = MK2MCS_R34BPSK;
            break;
          case 24:  // 12mbps
            frame_info->mcs = MK2MCS_R12QPSK;
            break;
          case 36:  // 18mbps
            frame_info->mcs = MK2MCS_R34QPSK;
            break;
          case 48:  // 24mbps
            frame_info->mcs = MK2MCS_R12QAM16;
            break;
          case 72:  // 36mbps
            frame_info->mcs = MK2MCS_R34QAM16;
            break;
          case 96:  // 48mbps
            frame_info->mcs = MK2MCS_R23QAM64;
            break;
          case 108:  // 54mbps
            frame_info->mcs = MK2MCS_R34QAM64;
            break;
          default:
            // printf("unknown modulation / coding scheme: %d, ",rti.this_arg_index);
            // print_mac(frame_info->src_address);
            // printf("\n");
            // assert(0);
            return 0;
        }
        break;

      case IEEE80211_RADIOTAP_FLAGS:
      {
        u8 flags = *rti.this_arg;

        if (flags & IEEE80211_RADIOTAP_F_FCS)
        {
          bytes -= FCS_LEN;
        }
      }
      break;

      case IEEE80211_RADIOTAP_DBM_ANTSIGNAL:
        frame_info->rcv_power = (float)*((char *)rti.this_arg);
        break;

      case IEEE80211_RADIOTAP_DBM_ANTNOISE:
        frame_info->noise_power = (float)*((char *)rti.this_arg);
        break;

      case IEEE80211_RADIOTAP_CHANNEL:
        prd.m_nChannel = le16_to_cpu(*((u16 *)rti.this_arg));
        prd.m_nChannelFlags = le16_to_cpu(*((u16 *)(rti.this_arg + 2)));
        break;

      case IEEE80211_RADIOTAP_RX_FLAGS:
      {
        frame_info->was_tx = false;
      }
      break;

      case IEEE80211_RADIOTAP_TX_FLAGS:
      {
        int flags = *rti.this_arg;
        frame_info->tx_successful = (flags & IEEE80211_RADIOTAP_F_TX_FAIL) != IEEE80211_RADIOTAP_F_TX_FAIL;
      }
      break;

      case IEEE80211_RADIOTAP_DATA_RETRIES:
        frame_info->tx_retries = *rti.this_arg;
        break;

      case IEEE80211_RADIOTAP_TSFT:
        frame_info->tx_tsft = le64_to_cpu(*((u64 *)rti.this_arg));
        break;
    }
  }

  frame_info->snr = frame_info->rcv_power - frame_info->noise_power;
  memcpy(frame_info->dst_address, ieee80211_get_DA((struct ieee80211_hdr *)&mac_hdr), sizeof(frame_info->dst_address));
  memcpy(frame_info->src_address, ieee80211_get_SA((struct ieee80211_hdr *)&mac_hdr), sizeof(frame_info->src_address));
  frame_info->channel_number = ieee80211_freq_to_ofdm_chan(5000, prd.m_nChannel);

  if ((!tx_okay) && frame_info->was_tx)
  {
    // we are just sniffing our own packet, just ignore it
    printf("Sniffing own packet\n");
    return 0;
  }

  if (bytes < max_size)
  {
    memcpy(payload, packet + u16HeaderLen + n80211HeaderLength + 8, bytes);
    return bytes;
  }
  else
  {
    printf("error: not enough space to copy payload\n");
    return 0;
  }
}

int set_channel(int fd, const char ifname[IFNAMSIZ], int channel)
{
  struct iwreq wrq;
  double freq;
  memset(&wrq, 0, sizeof(struct iwreq));

  // we want a fixed frequency
  wrq.u.freq.flags = IW_FREQ_FIXED;

  // get frequency from channel number
  // freq = (double) ieee80211_ofdm_chan_to_freq(5000, channel) * 1e6;
  freq = (double)ieee80211_ofdm_chan_to_freq(2407, channel) * 1e6;
  printf("freq: %f, channel: %d\n", freq, channel);
  // transform frequency into mantissa/exponent notation. from iwconfig
  wrq.u.freq.e = (short)(floor(log10(freq)));
  if (wrq.u.freq.e > 8)
  {
    wrq.u.freq.m = ((long)(floor(freq / pow(10, wrq.u.freq.e - 6)))) * 100;
    wrq.u.freq.e -= 8;
  }
  else
  {
    wrq.u.freq.m = (long)freq;
    wrq.u.freq.e = 0;
  }

  // Set device name
  strncpy(wrq.ifr_ifrn.ifrn_name, ifname, IFNAMSIZ);
  /* Do the request */
  int ret = ioctl(fd, SIOCSIWFREQ, &wrq);
  //    printf("set_channel: Request: %d, %lu, %s, %d, %dHz\n",fd, SIOCSIWFREQ, wrq.ifr_ifrn.ifrn_name, wrq.u.freq.m,
  //    wrq.u.freq.e);
  if (ret < 0)
  {
    perror("set frequency failed");
  }
  return ret;
}

int set_mcs(int fd, const char ifname[IFNAMSIZ], enum MK2MCS mcs)
{
  struct iwreq wrq;

  memset(&wrq, 0, sizeof(struct iwreq));

  // we want a fixed data rate
  wrq.u.bitrate.fixed = 1;

  switch (mcs)
  {
    case MK2MCS_R12BPSK:
      wrq.u.bitrate.value = 6e6;
      break;
    case MK2MCS_R34BPSK:
      wrq.u.bitrate.value = 9e6;
      break;
    case MK2MCS_R12QPSK:
      wrq.u.bitrate.value = 12e6;
      break;
    case MK2MCS_R34QPSK:
      wrq.u.bitrate.value = 18e6;
      break;
    case MK2MCS_R12QAM16:
      wrq.u.bitrate.value = 24e6;
      break;
    case MK2MCS_R34QAM16:
      wrq.u.bitrate.value = 36e6;
      break;
    case MK2MCS_R23QAM64:
      wrq.u.bitrate.value = 48e6;
      break;
    case MK2MCS_R34QAM64:
      wrq.u.bitrate.value = 54e6;
      break;
    default:
      printf("set_mcs: invalid modulation and coding scheme\n");
      exit(1);
      break;
  }

  // Set device name
  strncpy(wrq.ifr_ifrn.ifrn_name, ifname, IFNAMSIZ);
  /* Do the request */
  int ret = ioctl(fd, SIOCSIWRATE, &wrq);
  //  printf("set_mcs: Request: %d, %lu, %s, Bitrate: %d\n",fd, SIOCSIWRATE, wrq.ifr_ifrn.ifrn_name,
  //  wrq.u.bitrate.value);
  if (ret < 0)
  {
    perror("set mcs failed");
  }
  return ret;
}

int set_txpower(const char ifname[IFNAMSIZ], tMK2Power txpower)
{
  struct iwreq wrq;

  memset(&wrq, 0, sizeof(struct iwreq));

  // we want a fixed tx power
  wrq.u.txpower.fixed = 1;
  // the power is in dBm
  wrq.u.txpower.flags = IW_TXPOW_DBM;
  // set power value
  wrq.u.txpower.value = txpower;

  // Set device name
  strncpy(wrq.ifr_ifrn.ifrn_name, ifname, IFNAMSIZ);
  /* Do the request */
  // int ret = ioctl(fd, SIOCSIWTXPOW, &wrq);
  // HACKED to always send 1 as the return value//
  int ret = 1;
  if (ret < 0)
  {
    perror("set txpower");
  }
  return ret;
}

int get_mac_address(int fd, const char ifname[IFNAMSIZ], unsigned char mac[6])
{
  struct ifreq ifreq;

  memset(&ifreq, 0, sizeof(struct iwreq));

  // Set device name
  strncpy(ifreq.ifr_ifrn.ifrn_name, ifname, IFNAMSIZ);
  /* Do the request */
  int ret = ioctl(fd, SIOCGIFHWADDR, &ifreq);
  if (ret < 0)
  {
    perror("get mac address");
  }
  else
  {
    memcpy(mac, ifreq.ifr_ifru.ifru_hwaddr.sa_data, 6);
  }
  return ret;
}
