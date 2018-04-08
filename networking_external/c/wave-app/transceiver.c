/**
 * Defines helper function for seding and receiving 11p packet, independently
 * from the platform (i.e., Cohda or UNEX)
 */

#include "transceiver.h"

#ifdef PLATFORM_UNEX
#include "radiotap.h"
#include "ieee80211_radiotap.h"
#include "ieee80211.h"
#include "cohda_defs.h"
#include <linux/wireless.h>
#include <math.h>
#include <sys/ioctl.h>
#include <errno.h>
#endif

#include <stdio.h>

void dump_buffer(char* buf, int len) {
	int i;
	for (i=0; i < len; ++i) {
		char c = buf[i];
		if (c >= 'A') {
			printf("%c", buf[i]);
		} else {
			printf(".");
		}
	}
	printf("\n");
}

void print_mac(unsigned char mac[ETH_ALEN]) {
	printf("%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

//send and receive functions for cohda platform
#ifdef PLATFORM_COHDA

//----------------------------------------------FUNCTIONS BORROWED FROM COHDA--------------------------//
// local prototypes
static void Tx_Exit(tTx *pTx);

/**
 * @brief Initialize the Tx application
 * @param pTx handle to Tx Object with pointer to MLME_IF Handle
 * @param pTxOpts pointer Configuration Object
 * Set default values in the application's variables (the 'pTx' handle)
 */
void Tx_Init(tTx *pTx, tTxOpts * pTxOpts) {

	//pTx->Res = 0;
	pTx->Fd = -1; // File description invalid
	pTx->IfIdx = -1;
	pTx->SeqNum = 0;
	pTx->logFile = 0;

	// Create an output Buffer (could have large persistent Buf in pTx for speed)
	pTx->pBuf = (unsigned char *) malloc(0xFFFFU);
	if (pTx->pBuf == NULL ) {
		printf("Fail: malloc() errno %d\n", errno);
		//pTx->Res = errno;
		Tx_Exit(pTx);
	}
	pTx->pRxBuf = (unsigned char *) malloc(0xFFFFU);
	if (pTx->pRxBuf == NULL ) {
		printf("Fail: malloc() errno %d\n", errno);
		//pTx->Res = errno;
		Tx_Exit(pTx);
	}

	// PreLoad the Ethernet Header (used in RAW frames)
	memcpy(pTx->EthHdr.h_source, pTxOpts->SrcAddr, ETH_ALEN); // SA

//	// preload 802.11 header (used in MGMT frames)
//	pTx->Dot11Hdr.FrameControl = CpuToLe16(TEST_DOT11_FRAMECTL);
//	pTx->Dot11Hdr.DurationId = CpuToLe16(TEST_DOT11_DURATIONID);
//	memcpy(pTx->Dot11Hdr.Address2, pTxOpts->SrcAddr, ETH_ALEN); // SA
//	memset(pTx->Dot11Hdr.Address3, 0x33, ETH_ALEN); // BSSID
//	pTx->Dot11Hdr.SeqControl = CpuToLe16(TEST_DOT11_SEQCTL); // Sequence control info

	// Get internal quick Interface type from string
	if (strcmp("wave-raw", pTxOpts->pInterfaceName) == 0)
		pTx->InterfaceID = INTERFACEID_WAVERAW;
	else {
		printf("Fail: no such Interface %s\n", pTxOpts->pInterfaceName);
		//pTx->Res = errno;
		Tx_Exit(pTx);
	}

}

/**
 * @brief De-initialize the Tx application
 * @param pTx handle to Tx Object with pointers Socket and MLME_IF
 * Cleanup the application's variables (the 'pTx' handle)
 */
static void Tx_Exit(tTx *pTx) {

	if (pTx->Fd >= 0) {
		close(pTx->Fd);
		pTx->Fd = -1;
	}

	// free any allocated tx Buf
	if (pTx->pBuf != NULL ) {
		free(pTx->pBuf);
		pTx->pBuf = NULL;
	}
	if (pTx->pRxBuf != NULL ) {
		free(pTx->pRxBuf);
		pTx->pRxBuf = NULL;
	}

	//TODO
	//exit(pTx->Res);
}

/**
 * @brief Open a raw linux (AF_PACKET) socket to a specific network interface
 * @param pName The name of the network interface
 * @return the socket's "fd", negative values are errno values
 */
static int Tx_OpenSocket(const char *pName) {
	int Res = -ENOSYS;
	struct sockaddr_ll SocketAddress = { 0, };
	int SocketFd = -1;

	SocketFd = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
	if (SocketFd < 0) {
		printf("socket() failed errno %d '%s'\n", errno, strerror(errno));
		return -errno;
	}

	SocketAddress.sll_family = AF_PACKET;
	SocketAddress.sll_protocol = htons(ETH_P_ALL);
	SocketAddress.sll_ifindex = if_nametoindex(pName);

	Res = bind(SocketFd, (struct sockaddr *) &SocketAddress, sizeof(SocketAddress));
	if (Res < 0) {
		printf("bind() failed errno %d '%s'\n", errno, strerror(errno));
		close(SocketFd);
		return Res;
	}

	return SocketFd;
}

/**
 * @brief Open a raw linux (AF_PACKET) socket to the interface
 * @param pTx pointer to Tx Object owning Socket handle ("fd")
 */
static void Tx_OpenInterface(tTx * pTx, char * pInterfaceName) {
	pTx->Fd = Tx_OpenSocket(pInterfaceName);
	if (pTx->Fd < 0) {
		printf("Fail: open '%s'\n", pInterfaceName);
		Tx_Exit(pTx);
	}
	pTx->IfIdx = if_nametoindex(pInterfaceName);
}

/**
 * @brief Close the socket associated with the interface
 * @param pTx pointer to Tx Object owning Socket handle ("fd")
 */
static void Tx_CloseInterface(tTx * pTx) {
	if (pTx->Fd >= 0) {
		close(pTx->Fd);
		pTx->Fd = -1;
	} else {
		printf("Fail: close interface\n");
		Tx_Exit(pTx);
	}
}
//------------------------------------------END FUNCTIONS BORROWED FROM COHDA--------------------------//

typedef struct Tx_tag {
	/// MLME interface handle
	tMK2MLMEIFHandle MIFH;
	/// MLME_IF API function return code
	tMK2Status Res;
	/// wave-raw or wave-mgmt file descriptor
	int Fd;
	/// wave-raw or wave-mgmt if_index
	int IfIdx;
	/// Enum version of interface name
	tInterfaceID InterfaceID;
	/// Channel Type (CCH or SCH)
	tMK2Channel Channel;
} tCH;

void init_setup(tTxOpts *dev_params) {

	//parameters passed to MK2 API
	tMK2ChanProfile cch_profile, sch_profile;
	tMK2ChanProfile *p_cch_profile = &cch_profile, *p_sch_profile = &sch_profile;

	tCH ch;
	tCH *pCH = &ch;

	pCH->MIFH = NULL; // No MLME IF Handle Yet
	pCH->Fd = -1; // File description invalid
	pCH->IfIdx = -1;

	// Copy the requested Channel into the top level object
	//this should be CCH in our case, we don't do any channel switching
	pCH->Channel = MK2CHAN_CCH;

	// Get internal quick Interface type from string
	if (strcmp("wave-raw", dev_params->pInterfaceName) == 0)
		pCH->InterfaceID = INTERFACEID_WAVERAW;
	//	else if (strcmp("wave-mgmt", pCHOpts->pInterfaceName) == 0)
	//		pCH->InterfaceID = INTERFACEID_WAVEMGMT;
	//	else if (strcmp("wave-data", pCHOpts->pInterfaceName) == 0)
	//		pCH->InterfaceID = INTERFACEID_WAVEDATA;
	else {
		printf("Fail: no such Interface %s\n", dev_params->pInterfaceName);
	}

	assert(MK2_mlme_if_open(dev_params->pInterfaceName, &(pCH->MIFH)) == MK2STATUS_SUCCESS);

	switch (pCH->Channel) {
	case MK2CHAN_CCH:

		//stop cch. THIS IS F*****G IMPORTANT. if not called, card will use old profile and nothing will work
		assert(MK2_mlme_if_cchend(pCH->MIFH) == MK2STATUS_SUCCESS);

		//get current channel profiles, for CCH and SCH
		assert(MK2_mlme_if_getcch(pCH->MIFH, p_cch_profile) == MK2STATUS_SUCCESS);
		assert(MK2_mlme_if_getsch(pCH->MIFH, p_sch_profile) == MK2STATUS_SUCCESS);

		if (p_sch_profile->ChannelNumber == dev_params->TxCHOpts.ChannelNumber) {
			//SCH is set on the channel we want to set for CCH, so we have to change it
			//to avoid receiving messages twice, once from one radio, once from another
			int sch_channel;
			//change channel to another one
			if (dev_params->TxCHOpts.ChannelNumber != 172) {
				sch_channel = 172;
			} else {
				sch_channel = 178;
			}
			//stop sch
			assert(MK2_mlme_if_schend(pCH->MIFH) == MK2STATUS_SUCCESS);
			//change SCH channel
			p_sch_profile->ChannelNumber = sch_channel;
			assert(MK2_mlme_if_setsch(pCH->MIFH, p_sch_profile) == MK2STATUS_SUCCESS);
			//apply configuration to SCH, disabling channel switching
			assert(MK2_mlme_if_schstart(pCH->MIFH, false) == MK2STATUS_SUCCESS);
		}

		//set CCH channel parameters
		//channel number
		p_cch_profile->ChannelNumber = dev_params->TxCHOpts.ChannelNumber;
		//we want to receive and send only on one antenna. for now, understanding MIMO is not our business
		p_cch_profile->RxAntenna = MK2_TXANT_ANTENNA1;
		p_cch_profile->TxAntenna = MK2_TXANT_ANTENNA1;
		//set channel bandwidth, 10 MHz or 20 MHz
		p_cch_profile->Bandwidth = dev_params->TxCHOpts.Bandwidth;
		//do not use transmission power control. we set transmission power by ourself
		p_cch_profile->DefaultTPC = false;
		//the same is valid for transmission rate control. we choose modulation and coding scheme
		p_cch_profile->DefaultTRC = false;
		//since we are using one antenna, this parameter won't be used. when using more than one
		//antenna, it specifies whether transmission on one radio should be prevented when the
		//other is transmitting/receiving
		p_cch_profile->DualTxControl = MK2TXC_TXRX;
		//transmission power
		p_cch_profile->DefaultTxPower = dev_params->TxCHOpts.TxPower * 2;
		//default modulation and coding scheme
		p_cch_profile->DefaultMCS = MK2MCS_R12BPSK;

		//apply configuration to the radio
		assert(MK2_mlme_if_setcch(pCH->MIFH, p_cch_profile) == MK2STATUS_SUCCESS);

		//set the kind of ethernet type we want the interface to send up to the application layer
		tMK2InterfaceFilterSpec Filt;
		Filt.Cmd = MK2_CMD_SET;
		Filt.TableLen = 1;
		uint16_t etherFilter = htons(dev_params->TxCHOpts.EtherType);
		Filt.pFiltTable = &etherFilter;
		//apply the filter
		assert(MK2_mlme_if_filter(pCH->MIFH, &Filt) == MK2STATUS_SUCCESS);

		//start to use new profile
		assert(MK2_mlme_if_cchstart(pCH->MIFH) == MK2STATUS_SUCCESS);

		break;
//	case MK2CHAN_SCH:
//		pCH->Res = MK2_mlme_if_getcch(pCH->MIFH, p_cch_profile);
//		p_cch_profile->ChannelNumber = dev_params->TxCHOpts.ChannelNumber;
//		pCH->Res = MK2_mlme_if_setsch(pCH->MIFH, p_cch_profile);
//		break;
	default:
		pCH->Res = -1;
		break;
	}

	assert(MK2_mlme_if_close(&(pCH->MIFH)) == MK2STATUS_SUCCESS);

}

device_t *open_device(tTxOpts *params) {

	//transmission parameters
	tTxOpts TxOpts = *((tTxOpts*) params);
	tTxOpts* pTxOpts = &TxOpts;
	//device data
	device_t *pDevice = (device_t*) malloc(sizeof(device_t));
	//copy txopts into the descriptor
	pDevice->txArgs = TxOpts;
	//make a pointer to tx device handles
	tTx* pTx = &pDevice->tx;

	// Initialise the Socket to the device to default values
	Tx_Init(pTx, pTxOpts);

	// Open a raw socket to the specified network interface.
	Tx_OpenInterface(pTx, pTxOpts->pInterfaceName);

	return pDevice;

}

void close_device(device_t *device) {
	tTx* pTx = &device->tx;
	Tx_CloseInterface(pTx);
	Tx_Exit(pTx);
	free(device);
}

int send_frame(device_t* device, void *payload, int size, long txPower) {

	tTx *pTx = &device->tx;
	tTxOpts *pTxOpts = &device->txArgs;

	int ThisFrameLen, ThisBytesSent; // Number of Bytes
	char *pBuf;
	unsigned char *pPayload;
	struct MK2TxDescriptor *pTxDesc;
	struct ethhdr *pEthHdr;
	long TxPower;
	tTxCHOpts * pTxCHOpts;

	//this buffer will include tx descriptor (with channel, modulation and coding scheme, ...),
	//ethernet header, and the payload. it is pre-allocated and stored into pTx to save time
	pBuf = (char *) (pTx->pBuf);
	pTxCHOpts = &(pTxOpts->TxCHOpts);

	assert(pBuf != NULL);
	assert(pTxCHOpts != NULL);

	//first part of the transmission buffer is the tx descriptor
	pTxDesc = (tMK2TxDescriptor *) ((char *) pBuf);

	//just send on wave-raw
	assert(pTx->InterfaceID == INTERFACEID_WAVERAW);

	//--------------------------------------------------------------------------
	// WAVE-RAW frame: | TxDesc | Eth Header | Protocol & Payload |
	pEthHdr = (struct ethhdr *) ((char *) pTxDesc + sizeof(tMK2TxDescriptor));
	pPayload = (unsigned char *) ((char *) pEthHdr + sizeof(struct ethhdr));

	//copy the ethernet header
	// Ethernet Header (SA is already in from Tx_Init())
	memcpy(pEthHdr->h_source, pTx->EthHdr.h_source, ETH_ALEN); // SA
	memcpy(pEthHdr->h_dest, pTxCHOpts->DestAddr, ETH_ALEN); // DA
	pEthHdr->h_proto = htons(pTxCHOpts->EtherType); // EtherType

	pTx->SocketAddress.sll_protocol = htons(pTxCHOpts->EtherType);

	//TxPower = pTxCHOpts->TxPower * 2;
	TxPower = txPower * 2;

	//copy all the necessary info into the tx descriptor at the beginning of the buffer
	pTxDesc->ChannelNumber = pTxCHOpts->ChannelNumber;
	pTxDesc->Priority = pTxCHOpts->Priority;
	pTxDesc->Service = pTxCHOpts->Service;
	pTxDesc->TxAntenna = pTxCHOpts->pTxAntenna;
	pTxDesc->Expiry = 0; //never
	pTxDesc->MCS = pTxCHOpts->pMCS;
	pTxDesc->TxPower.ManualPower = (tMK2Power) TxPower;
	pTxDesc->TxPower.PowerSetting = MK2TPC_MANUAL;

	// Setup the user data
	ThisFrameLen = size + ((unsigned int) pPayload - (unsigned int) pBuf);

	//finally copy the payload
	memcpy(pPayload, payload, size);

	//send frame down to the device. have a good flight baby!
	ThisBytesSent = sendto(pTx->Fd, pBuf, ThisFrameLen, 0, NULL, 0);
	assert(ThisBytesSent == ThisFrameLen);

	return ThisBytesSent;
}

int receive_frame(device_t *device, void *payload, int max_size, struct frame_info_t *frame_info, u8 tx_okay) {

	tTx *pTx = &device->tx;
	//number of bytes returned when reading the device, and size of the actual payload
	int count, payload_len;
	//buffer used to store received data
	unsigned char * pBuf;
	//cohda rx descriptor (containing rx power, channel, etc.)
	tMK2RxDescriptor * pRxDesc;
	//pointer to the actual payload inside the receive
	unsigned char *pPayload;
	//source MAC address
	unsigned char * pRxSrcAddr, *pRxDstAddr;
	;

	assert(pTx != NULL);

	//quick access to space for RxBuf, Ethernet Header and MK2RxDesc
	pBuf = pTx->pRxBuf;
	//pBuf shoud be pre-initialized, is it really like that?
	assert(pBuf != NULL);

	//instead of using recvfrom, which is blocking, we use select()
	//using select, it is possible to specify a timeout for the receive operation
	struct timeval stTimeOut;
	//timeout set to 5 second
	stTimeOut.tv_sec = 5;
	stTimeOut.tv_usec = 0;
	//file descriptors to be watched (actually only one, our device)
	fd_set stReadFDS;
	//set no FDs in the set
	FD_ZERO(&stReadFDS);
	//add the device file descriptor to the set of read FDs
	FD_SET(pTx->Fd, &stReadFDS);

	//wait for incoming data or for a timeout
	count = select(pTx->Fd + 1, &stReadFDS, NULL, NULL, &stTimeOut);

	if (count <= 0) {
		//timeout is expired or something went wrong, nothing to read
		return count;
	} else {
		if (FD_ISSET(pTx->Fd, &stReadFDS)) {
			//there is data to be read. do that with recv()
			count = recv(pTx->Fd, pBuf, -1, 0);
		} else {
			printf("this should never happen\n");
			//WTF??
			assert(0);
		}
	}

	//Make sure the RxFrame is at least minimal length
	assert(count > (sizeof(tMK2RxDescriptor) + sizeof(struct ethhdr)));

	//Extract MK2RxDesc and Payload
	pRxDesc = (tMK2RxDescriptor *) ((unsigned char *) pBuf);

	switch (pTx->InterfaceID) {
	case INTERFACEID_WAVERAW:
		//--------------------------------------------------------------------------
		// WAVE-RAW frame: | RxDesc | Eth Header | Payload |
		pPayload = (unsigned char *) pBuf; // start of Buffer
		pPayload += sizeof(tMK2RxDescriptor); // skip Rx Desc to Ethernet Header
		pRxSrcAddr = (unsigned char*) ((struct ethhdr *) pPayload)->h_source; // get the Source MAC to match
		pRxDstAddr = (unsigned char*) ((struct ethhdr *) pPayload)->h_dest; // get the Source MAC to match

		pPayload += sizeof(struct ethhdr); // skip Ethernet Header

		break;
	default:
		fprintf(stderr, "Fail: Invalid Interface\n");
		return -1;
		break;
	}

	// Use pointer arithemtic to get payload length from total
	// -4 is to remove 4 additional bytes which are added at the end by the driver
	// the source is still unknown, but it seems to be the MAC CRC which is just
	// sent up
	payload_len = count - ((unsigned int) pPayload - (unsigned int) pBuf) - 4;

	//copy payload to user buffer, avoiding out of memory errors
	memcpy(payload, pPayload, payload_len <= max_size ? payload_len : max_size);

	//copy info about received frame
	frame_info->noise_power = pRxDesc->RxNoiseA / 2.0;
	frame_info->rcv_power = pRxDesc->RxPowerA / 2.0;
	frame_info->snr = frame_info->rcv_power - frame_info->noise_power;
	frame_info->channel_number = pRxDesc->ChannelNumber;
	frame_info->mcs = pRxDesc->MCS;
	memcpy(frame_info->src_address, pRxSrcAddr, ETH_ALEN);
	memcpy(frame_info->dst_address, pRxDstAddr, ETH_ALEN);

	return payload_len <= max_size ? payload_len : max_size;
}

#endif

//send and receive functions for unex platform
#ifdef PLATFORM_UNEX

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
int set_txpower(int fd, const char ifname[IFNAMSIZ], tMK2Power txpower);
/**
 * Get hw mac address
 */
int get_mac_address(int fd, const char ifname[IFNAMSIZ], unsigned char mac[6]);

void init_setup(tTxOpts *dev_params) {
	//nothing to do. we cannot change channel, MCS, etc., until the interface has been opened
}

device_t* open_device(tTxOpts *params) {

	device_t *device = malloc(sizeof(device_t));
	tTx *pTx = &device->tx;
	device->txArgs = *params;
	tTxOpts *pTxOpts = &device->txArgs;

	char *device_name = params->pInterfaceName;

	pTx->Fd = -1; // File description invalid
	pTx->IfIdx = -1;
	pTx->SeqNum = 0;
	pTx->logFile = 0;

	// Create an output Buffer (could have large persistent Buf in pTx for speed)
	pTx->pBuf = (unsigned char *) malloc(0xFFFFU);
	if (pTx->pBuf == NULL ) {
		printf("Fail: malloc() errno %d\n", errno);
	}
	pTx->pRxBuf = (unsigned char *) malloc(0xFFFFU);
	if (pTx->pRxBuf == NULL ) {
		printf("Fail: malloc() errno %d\n", errno);
	}

	// PreLoad the Ethernet Header (used in RAW frames)
	memcpy(pTx->EthHdr.h_source, pTxOpts->SrcAddr, ETH_ALEN); // SA

	char *errbuf = malloc(sizeof(char) * PCAP_ERRBUF_SIZE);
	pTx->pcap = pcap_open_live(device_name, BUFSIZ, 1, 1000, errbuf);
	if (!pTx->pcap) {
		fprintf(stderr, "Couldn't open device %s: %s\n", device_name, errbuf);
		return 0;
	}

	pTx->Fd = socket(AF_INET, SOCK_DGRAM, 0);
	set_channel(pTx->Fd, device_name, pTxOpts->TxCHOpts.ChannelNumber);
	set_mcs(pTx->Fd, device_name, pTxOpts->TxCHOpts.pMCS);
	set_txpower(pTx->Fd, device_name, pTxOpts->TxCHOpts.TxPower);
	get_mac_address(pTx->Fd, device_name, pTxOpts->SrcAddr);

	free(errbuf);

	return device;
}

void close_device(device_t *device) {
	close(device->tx.Fd);
	pcap_close(device->tx.pcap);
}

int send_frame(device_t *device, void *payload, int size, long txPower) {

	//buffer where to store the data (radiotap, wifi header, payload, etc.)
	u8 u8aSendBuffer[2000];
	struct ieee80211_qos_hdr mac_hdr;

	//reset memory content
	memset(u8aSendBuffer, 0, sizeof(u8aSendBuffer));
	//make a pointer to the buffer
	u8* pu8 = u8aSendBuffer;

	//insert radiotap header at the beginning, for nothing. it is ignored anyway
	memcpy(u8aSendBuffer, u8aRadiotapHeader, sizeof(u8aRadiotapHeader));
	//force the rate to use. the 5th is 12*2, which means 12Mbps in the 20MHz band
	//or 6Mbps in the 10MHz band
	pu8[OFFSET_RATE] = u8aRatesToUse[5];
	pu8[OFFSET_TXPOWER] = 20;
	pu8[OFFSET_ANTENNA] = 1;
	pu8 += sizeof(u8aRadiotapHeader);

	//set up source address
	memcpy(&mac_hdr, wifi_hdr, sizeof(wifi_hdr));
	memcpy(mac_hdr.addr2, device->txArgs.SrcAddr, 6);

	memcpy(pu8, &mac_hdr, sizeof(struct ieee80211_qos_hdr));
	pu8 += sizeof(struct ieee80211_qos_hdr);
	memcpy(pu8, llc_hdr, sizeof(llc_hdr));
	pu8 += sizeof(llc_hdr);

	//add payload
	memcpy(pu8, payload, size);
	pu8 += size;

	//set tx power just before sending
	set_txpower(device->tx.Fd, device->txArgs.pInterfaceName, txPower);

	int r = pcap_inject(device->tx.pcap, u8aSendBuffer, pu8 - u8aSendBuffer);
	if (r != (pu8 - u8aSendBuffer)) {
		perror("Trouble injecting packet");
		return 0;
	}

	return size;
}

int receive_frame(device_t *device, void *payload, int max_size, struct frame_info_t *frame_info, u8 tx_okay) {

	struct pcap_pkthdr hdr;
	const unsigned char *packet;
	struct ieee80211_radiotap_iterator rti;
	struct ieee80211_hdr_3addr mac_hdr;
	u16 u16HeaderLen;
	u16 n80211HeaderLength;
	int bytes;
	PENUMBRA_RADIOTAP_DATA prd;

	char errbuf[PCAP_ERRBUF_SIZE];

	//ensure that pcap_next call is blocking
	pcap_setnonblock(device->tx.pcap, 0, errbuf);
	//wait for next frame captured from the device
	if ((packet = pcap_next(device->tx.pcap, &hdr)) == 0) {
		return 0;
	}

	//get radiotap header length
	u16HeaderLen = (packet[2] + (packet[3] << 8));
	if (hdr.len < u16HeaderLen)
		return 0;

	//we still don't know whether frame is data or qos data. for now cast to data which is enough
	mac_hdr = *(struct ieee80211_hdr_3addr *) (packet + u16HeaderLen);

	//check frame type
	if (ieee80211_is_data(mac_hdr.frame_control)) {
		n80211HeaderLength = sizeof(struct ieee80211_hdr_3addr);
		if (ieee80211_is_data_qos(mac_hdr.frame_control)) {
			n80211HeaderLength = sizeof(struct ieee80211_qos_hdr);
		}
	} else {
		printf("received something that is not a data frame\n");
		//we are only interested in data frames
		return 0;
	}

	//compute psdu size
	bytes = hdr.len - (u16HeaderLen + n80211HeaderLength + 8);

	//init radiotap fields iterator
	if (ieee80211_radiotap_iterator_init(&rti, (struct ieee80211_radiotap_header *) packet, bytes) < 0)
		return 0;

	// if the frame does not come with IEEE80211_RADIOTAP_RX_FLAGS, assume it was transmitted
	frame_info->was_tx = true;

	//loop through all radiotap fields available
	while ((ieee80211_radiotap_iterator_next(&rti)) == 0) {

		switch (rti.this_arg_index) {
		case IEEE80211_RADIOTAP_RATE:
			prd.m_nRate = (*rti.this_arg);
			frame_info->mcs = 0;
			switch (prd.m_nRate) {
			case 12: //6mbps
				frame_info->mcs = MK2MCS_R12BPSK;
				break;
			case 18: //9mbps
				frame_info->mcs = MK2MCS_R34BPSK;
				break;
			case 24: //12mbps
				frame_info->mcs = MK2MCS_R12QPSK;
				break;
			case 36: //18mbps
				frame_info->mcs = MK2MCS_R34QPSK;
				break;
			case 48: //24mbps
				frame_info->mcs = MK2MCS_R12QAM16;
				break;
			case 72: //36mbps
				frame_info->mcs = MK2MCS_R34QAM16;
				break;
			case 96: //48mbps
				frame_info->mcs = MK2MCS_R23QAM64;
				break;
			case 108: //54mbps
				frame_info->mcs = MK2MCS_R34QAM64;
				break;
			default:
				printf("unknown modulation / coding scheme\n");
				assert(0);
				break;
			}
			break;

		case IEEE80211_RADIOTAP_FLAGS:
			{
				u8 flags = *rti.this_arg;

				if (flags & IEEE80211_RADIOTAP_F_FCS) {
					bytes -= FCS_LEN;
				}
			}
			break;

		case IEEE80211_RADIOTAP_DBM_ANTSIGNAL:
			frame_info->rcv_power = (float) *((char *) rti.this_arg);
			break;

		case IEEE80211_RADIOTAP_DBM_ANTNOISE:
			frame_info->noise_power = (float) *((char *) rti.this_arg);
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
	memcpy(frame_info->dst_address, ieee80211_get_DA((struct ieee80211_hdr *) &mac_hdr), sizeof(frame_info->dst_address));
	memcpy(frame_info->src_address, ieee80211_get_SA((struct ieee80211_hdr *) &mac_hdr), sizeof(frame_info->src_address));
	frame_info->channel_number = ieee80211_freq_to_ofdm_chan(5000, prd.m_nChannel);

	if ((!tx_okay) && frame_info->was_tx) {
		//we are just sniffing our own packet, just ignore it
		return 0;
	}

	if (bytes < max_size) {
		memcpy(payload, packet + u16HeaderLen + n80211HeaderLength + 8, bytes);
		return bytes;
	} else {
		printf("error: not enough space to copy payload\n");
		return 0;
	}

}

int set_channel(int fd, const char ifname[IFNAMSIZ], int channel) {

	struct iwreq wrq;
	double freq;
	memset(&wrq, 0, sizeof(struct iwreq));

	// we want a fixed frequency
	wrq.u.freq.flags = IW_FREQ_FIXED;

	// get frequency from channel number
	freq = (double) ieee80211_ofdm_chan_to_freq(5000, channel) * 1e6;

	// transform frequency into mantissa/exponent notation. from iwconfig
	wrq.u.freq.e = (short) (floor(log10(freq)));
	if (wrq.u.freq.e > 8) {
		wrq.u.freq.m = ((long) (floor(freq / pow(10, wrq.u.freq.e - 6)))) * 100;
		wrq.u.freq.e -= 8;
	} else {
		wrq.u.freq.m = (long) freq;
		wrq.u.freq.e = 0;
	}

	// Set device name
	strncpy(wrq.ifr_ifrn.ifrn_name, ifname, IFNAMSIZ);
	/* Do the request */
	int ret = ioctl(fd, SIOCSIWFREQ, &wrq);
	if (ret < 0) {
		perror("set frequency");
	}
	return ret;

}

int set_mcs(int fd, const char ifname[IFNAMSIZ], enum MK2MCS mcs) {

	struct iwreq wrq;

	memset(&wrq, 0, sizeof(struct iwreq));

	// we want a fixed data rate
	wrq.u.bitrate.fixed = 1;

	switch (mcs) {

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
		printf("%s invalid modulation and coding scheme\n", __FUNCTION__);
		exit(1);
		break;
	}

	// Set device name
	strncpy(wrq.ifr_ifrn.ifrn_name, ifname, IFNAMSIZ);
	/* Do the request */
	int ret = ioctl(fd, SIOCSIWRATE, &wrq);
	if (ret < 0) {
		perror("set mcs");
	}
	return ret;

}

int set_txpower(int fd, const char ifname[IFNAMSIZ], tMK2Power txpower) {

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
	int ret = ioctl(fd, SIOCSIWTXPOW, &wrq);
	if (ret < 0) {
		perror("set mcs");
	}
	return ret;

}

int get_mac_address(int fd, const char ifname[IFNAMSIZ], unsigned char mac[6]) {

	struct ifreq ifreq;

	memset(&ifreq, 0, sizeof(struct iwreq));

	// Set device name
	strncpy(ifreq.ifr_ifrn.ifrn_name, ifname, IFNAMSIZ);
	/* Do the request */
	int ret = ioctl(fd, SIOCGIFHWADDR, &ifreq);
	if (ret < 0) {
		perror("set mcs");
	} else {
		memcpy(mac, ifreq.ifr_ifru.ifru_hwaddr.sa_data, 6);
	}
	return ret;

}

#endif

