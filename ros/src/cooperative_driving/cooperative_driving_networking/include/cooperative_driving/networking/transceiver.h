//////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2017, CCS Labs
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the <organization> nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////////
/**
 * Defines helper function for seding and receiving 11p packet, independently
 * from the platform (i.e., Cohda or UNEX)
 */

#ifndef _TRANSCEIVER_H_
#define _TRANSCEIVER_H_

#include <arpa/inet.h>
#include <assert.h>
#include <errno.h>
#include <getopt.h>
#include <linux/if_ether.h>
#include <netpacket/packet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include <linux/if.h>

#include <pcap/pcap.h>
#include "cohda_defs.h"

typedef signed char s8;
typedef signed short s16;
typedef signed int s32;
typedef signed long long s64;
typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;
typedef unsigned long long u64;

#if defined(__BIG_ENDIAN__) || defined(_BIG_ENDIAN)

#define le32_to_cpu(x)                                                                                                 \
  ((((x) >> 24) & 0xff) | (((x) >> 8) & 0xff00) | (((x) << 8) & 0xff0000) | (((x) << 24) & 0xff000000))

#define le16_to_cpu(x) ((((x) >> 8) & 0xff) | (((x) << 8) & 0xff00))

#else

#define le64_to_cpu(x) (x)
#define le32_to_cpu(x) (x)
#define le16_to_cpu(x) (x)

#define cpu_to_le32(x) le32_to_cpu(x)
#define cpu_to_le16(x) le16_to_cpu(x)

#endif

#define unlikely(x) (x)

#define MAX_PENUMBRA_INTERFACES 8

/* wifi bitrate to use in 500kHz units */
static const u8 u8aRatesToUse[] = { 54 * 2, 48 * 2, 36 * 2, 24 * 2, 18 * 2, 12 * 2, 9 * 2, 11 * 2,
                                    11,  // 5.5
                                    2 * 2,  1 * 2 };

/* this is the template radiotap header we send packets out with */
static const u8 u8aRadiotapHeader[] = {
  0x00, 0x00,              // <-- radiotap version
  0x0b, 0x00,              // <- radiotap header length
  0x04, 0x0c, 0x00, 0x00,  // <-- bitmap
  0x6c,                    // <-- rate
  0x0c,                    //<-- tx power
  0x01                     //<-- antenna
};
#define OFFSET_RATE 8
#define OFFSET_TXPOWER 9
#define OFFSET_ANTENNA 10

static const char wifi_hdr[] = { '\x88', '\x00', '\x30', '\x00', '\xff', '\xff', '\xff', '\xff', '\xff',
                                 '\xff', '\x23', '\x23', '\x23', '\x23', '\x23', '\x23', '\xff', '\xff',
                                 '\xff', '\xff', '\xff', '\xff', '\xc0', '\x20', '\x20', '\x00' };

static const char llc_hdr[] = { '\xaa', '\xaa', '\x03', '\x00', '\x00', '\x00', '\x88', '\xb5' };

static const char wsm_hdr[] = { '\x02', '\x55', '\x04', '\x01', '\x40', '\x10', '\x01', '\x0c', '\x0f', '\x01',
                                '\xb2', '\x50', '\x00', '\x20', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00',
                                '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00',
                                '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00',
                                '\x00', '\x00', '\x00', '\x00', '\x00', '\x00' };

// this is where we store a summary of the
// information from the radiotap header

typedef struct
{
  int m_nChannel;
  int m_nChannelFlags;
  int m_nRate;
  int m_nAntenna;
  int m_nRadiotapFlags;
} __attribute__((packed)) PENUMBRA_RADIOTAP_DATA;

void dump_buffer(char *buf, int len);
void print_mac(unsigned char mac[ETH_ALEN]);

///**
// * Transmission parameters
// */
typedef struct TxCHOpts_tag
{
  /// Indicate the channel number that should be used (e.g. 172)
  tMK2ChannelNumber ChannelNumber;
  /// Indicate the priority to used
  tMK2Priority Priority;
  /// Indicate the 802.11 Service Class to use
  tMK2Service Service;
  /// Indicate the MCS to be used (may specify default or TRC)
  tMK2MCS pMCS;
  /// Indicate the power to be used in Manual mode. (NOTICE: dBm, not half dBm for Cohda)
  /// Its only used by the followers when sending DATA beacons
  long TxPower;
  /// Indicate the power the leader tells the others to use for data packets (NOTICE: dBm, not half dBm for Cohda)
  long TxPowerReduced;
  /// Indicate the antenna upon which packet should be transmitted
  tMK2TxAntenna pTxAntenna;
  /// Indicate the bandwidth to be used
  tMK2Bandwidth Bandwidth;
  /// Packet length in Bytes, list of ranges
  long PacketLength;
  /// DA to use in test packets to WAVE-RAW
  tMK2MACAddr DestAddr;
  /// Ethernet Type to use in Ethernet Header input to WAVE_RAW
  uint16_t EtherType;
} tTxCHOpts;

/**
 * Tx configuration
 */
typedef struct TxOpts_tag
{
  /// Channel Options
  tTxCHOpts TxCHOpts;
  /// SA to use in test packets (if supplied, driver default used otherwise)
  tMK2MACAddr SrcAddr;
  /// Interface (Wave Raw or Mgmt)
  char *pInterfaceName;
  /// beacon interval in milliseconds
  int beaconInterval;
  /// experiment id
  int experimentId;
  /// run number
  int runNumber;
  /// total number of packets to be sent
  int nPackets;
  /// directory were to store output files
  char *outDirectory;
  // which beaconing scheme to use (static or slotted)
  int beaconScheme;
  // offset to wait before sending beacon (only with slotted beaconing scheme)
  int beaconOffset;
  // Sleeptime to wait before sending first beacon (only with slotted beaconing scheme)
  int beaconSleeptime;
  // id, 0 leader, >=1 follower
  int id;
  // number of participants, leader excluded
  int nParticipants;
} tTxOpts;

typedef enum { INTERFACEID_WAVERAW = 0, INTERFACEID_WAVEDATA = 1, INTERFACEID_WAVEMGMT = 2 } eInterfaceID;
typedef uint8_t tInterfaceID;

typedef struct Tx
{
  /// MLME_IF API function return code
  //  tMK2Status Res;
  /// wave-raw file descriptor
  int Fd;
  /// wave-raw if_index
  int IfIdx;
  /// Enum version of interface name
  tInterfaceID InterfaceID;
  /// Socket address
  struct sockaddr_ll SocketAddress;
  /// Tx Buffer (rather than re-alloc each time)
  unsigned char *pBuf;
  /// Rx Buffer
  unsigned char *pRxBuf;
  /// Ethernet header (for preloading invariants)
  struct ethhdr EthHdr;
  //  /// 802.2 SNAP header (for preloading invariants)
  //  struct SNAPHeader Dot2Hdr;
  //  /// 802.11 MAC header (for preloading invariants)
  //  struct IEEE80211MACHdr Dot11Hdr;
  /// Tx descriptor
  //  struct MK2TxDescriptor TxDesc;
  /// Unique Identifier for each packet (Sequence Number)
  uint32_t SeqNum;
  /// output file
  FILE *logFile;
  /// pcap interface for unex device
  pcap_t *pcap;
} tTx;

/**
 * Struct used to include tTx and tTxOpts
 */
typedef struct tDeviceDescriptor_tag
{
  tTx tx;
  tTxOpts txArgs;
} device_t;

/**
 * define information about received frames. TBD: whether useful or not
 */
struct frame_info_t
{
  float snr;                            // dB
  float rcv_power;                      // dBm
  float noise_power;                    // dBm
  uint8_t channel_number;               // channel number
  enum MK2MCS mcs;                      // modulation and coding scheme
  unsigned char src_address[ETH_ALEN];  // source MAC address
  unsigned char dst_address[ETH_ALEN];  // destination MAC address
  u8 was_tx;                            // nonzero, if this is an report on a packet we sent
  u8 tx_successful;                     // only if was_tx: nonzero if packet was TX-ed successfully
  u8 tx_retries;                        // only if was_tx: number of retries before packet was TX-ed
  u64 tx_tsft;                          // only if was_tx: value of TSFT field
  //...
};

// send and receive functions

/**
 * Open device for sending and receiving.
 * \param params parameters needed for opening the device.
 * \return a pointer to the structure keeping info about
 * the opened device. The type of the structure depends
 * on the platform.
 */
device_t *open_device(tTxOpts *params);
/**
 * Closes a device used for sending and receiving
 * \param device device to be de-initialized, returned by open_device()
 */
void close_device(device_t *device);
/**
 * Send the desired payload
 * \param device device to be used to send, returned by open_device()
 * \param payload the payload to be sent
 * \param size size of the payload in bytes
 */
int send_frame(device_t *device, void *payload, int size, long txPower);
/**
 * Receive frame function. This function uses the select() function,
 * with a timeout of 5s. This function can be invoked in a polling
 * fashion and, if no data can be received, it returns after 5 seconds.
 * Return value can be checked to understand whether some data have been
 * received or not.
 *
 * \param device device where to receive frames from
 * \param payload where to store payload of received message
 * \param max_size alloced memory for payload. if payload is not
 * big enough, this will avoid out of memory errors
 * \param frame_info pointer to a struct where to store statistics
 * about received frame, such as power, SNR, ...
 * \return the size of the received payload, if something has been received,
 * 0 if there is nothing to be received, and a negative value if something
 * went wrong
 */
int receive_frame(device_t *device, void *payload, int max_size, struct frame_info_t *frame_info, u8 tx_okay);

#endif
