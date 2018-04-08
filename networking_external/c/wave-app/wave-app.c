

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <poll.h>
#include <string.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <signal.h>
#include <assert.h>
#include <getopt.h>

#include <sys/time.h> // for rate control
#include <sys/socket.h>
#include <netpacket/packet.h>
#include <net/ethernet.h>
#include <netinet/in.h>
#include <pthread.h>

#include "utils.h"

//transceiver functionalities
#include "transceiver.h"


int run = 1;

void txsignalhandler (int sig)
{
   // unlock only infinite loop
   run = 0;
}

void *receive_thread(void* params) {
	device_t* device = (device_t*)params;

	static int seqno = 0;

	char rcv_payload[2000];
	struct frame_info_t frame_info;
	int n;

	while (run) {
		memset(rcv_payload, 0, 2000);
		n = receive_frame(device, rcv_payload, 2000, &frame_info, 1);

		if (n < 0) break;
		if (n == 0) continue;

		dump_buffer(rcv_payload, n);

		// get current time
		struct timeval tv_r;
		gettimeofday(&tv_r, 0);

		seqno++;
		if (frame_info.was_tx) {
			printf("                                                                               ");
		}
		printf("%ld.%06ld: %s packet %5d took %9lld ns with %d retries status %s power %f noise %f\n", tv_r.tv_sec, tv_r.tv_usec, (frame_info.was_tx?"tx":"rx"), seqno, frame_info.tx_tsft, frame_info.tx_retries, (frame_info.tx_successful?"OK":"ERR"), frame_info.rcv_power, frame_info.noise_power);

	}

	return 0;
}

int main (int argc, char* argv[]) {

	signal(SIGTERM, &txsignalhandler);
	signal(SIGINT, &txsignalhandler);
	signal(SIGABRT, &txsignalhandler);

	tTxOpts tx_options;
	tMK2MACAddr src_mac = { 0x04, 0xe5, 0x48, 0x00, 0x10, 0x00 };
	tMK2MACAddr dst_mac = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
	tx_options.TxCHOpts.ChannelNumber = 1;
	tx_options.TxCHOpts.Priority = MK2_PRIO_0;
	tx_options.TxCHOpts.Service = MK2_QOS_NOACK;
	tx_options.TxCHOpts.pMCS = MK2MCS_R12QAM16;//MK2MCS_R12BPSK;
	tx_options.TxCHOpts.TxPower = 20;
	tx_options.TxCHOpts.pTxAntenna = MK2_TXANT_DEFAULT;
	tx_options.TxCHOpts.Bandwidth = MK2BW_20MHz;
	memcpy(tx_options.TxCHOpts.DestAddr, dst_mac, sizeof tx_options.TxCHOpts.DestAddr);
	tx_options.TxCHOpts.EtherType = 0x88b5;
	memcpy(tx_options.SrcAddr, src_mac, sizeof tx_options.SrcAddr);
	tx_options.pInterfaceName = "mon0";

	init_setup(&tx_options);
	device_t* device = (device_t *) open_device(&tx_options);

	pthread_t receive_thread_id;
	pthread_create(&receive_thread_id, NULL, receive_thread, (void *)device);

	char packet_buffer[150];
	memcpy(packet_buffer, "HELLO WORLD --- HELLO WORLD --- HELLO WORLD", 32);
	int packet_len = 32;

	while (run) {

		// sleep
		struct timespec time[1];
		time[0].tv_sec = 0;
		time[0].tv_nsec = (10 * 1000* 1000 ); // 500 ms
		nanosleep(time, NULL);

		// send
		printf("sending packet...\n");
		send_frame(device, packet_buffer, packet_len, tx_options.TxCHOpts.TxPower);

	}

	close_device(device);

	return 0;
}

