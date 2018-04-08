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
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <net/ethernet.h>
#include <netinet/in.h>
#include <netpacket/packet.h>
#include <pthread.h>
#include <sys/socket.h>
#include <sys/time.h>  // for rate control

// #include <functional>
// #include <string>

#include "util.h"

//#include <ros/ros.h>

// transceiver functionalities

// extern "C" {
#include "transceiver.h"
//}

// class Sender {
// public:
// 	Sender(std::string name) {
// 		tMK2MACAddr src_mac = { 0x18, 0xa6, 0xf7, 0x1c, 0x99, 0x90 };
// 		tMK2MACAddr dst_mac = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
// 		tx_options_.TxCHOpts.ChannelNumber = 1;
// 		tx_options_.TxCHOpts.Priority = MK2_PRIO_0;
// 		tx_options_.TxCHOpts.Service = MK2_QOS_NOACK;
// 		tx_options_.TxCHOpts.pMCS = MK2MCS_R12QAM16;//MK2MCS_R12BPSK;
// 		tx_options_.TxCHOpts.TxPower = 20;
// 		tx_options_.TxCHOpts.pTxAntenna = MK2_TXANT_DEFAULT;
// 		tx_options_.TxCHOpts.Bandwidth = MK2BW_20MHz;
// 		memcpy(tx_options_.TxCHOpts.DestAddr, dst_mac, sizeof tx_options_.TxCHOpts.DestAddr);
// 		tx_options_.TxCHOpts.EtherType = 0x88b5;
// 		memcpy(tx_options_.SrcAddr, src_mac, sizeof tx_options_.SrcAddr);
// 		tx_options_.pInterfaceName =(char*) "mon0";

// 		device_ = (device_t *) open_device(&tx_options_);

// 		//pthread_t receive_thread_id;
// 		//pthread_create(&receive_thread_id, NULL, receive_thread, (void *)device);

// 		//packet_len_ = name.length();
// 		packet_len_ = 32;
// //		memcpy(packet_buffer_, name.c_str(), packet_len_);
// 		memcpy(packet_buffer_, "HELLO WORLD --- HELLO WORLD --- HELLO WORLD", 32);

// //		int rate=1;
// 		//send_timer_ = nh_.createTimer(ros::Rate(rate), &Sender::send_cb, this);
// //		recv_timer_ = nh_.createTimer(ros::Rate(rate), &Sender::receive_cb, this);
// 	}

// 	~Sender() {
// 		close_device(device_);
// 	}

// 	void send () {

// 			send_frame(device_, packet_buffer_, packet_len_, tx_options_.TxCHOpts.TxPower);
// 	}
// private:
// 	// void send_cb(const ros::TimerEvent &timerEvent) {
// 	// 	printf("sending packet (buffer: %s)...\n",packet_buffer_);
// 	// 	send_frame(device_, packet_buffer_, packet_len_, tx_options_.TxCHOpts.TxPower);
// 	// }

// 	// void receive_cb(const ros::TimerEvent &timerEvent) {
// 	// 	static int seqno = 0;
// 	// 	printf("rcv\n");
// 	// 	char rcv_payload[2000];
// 	// 	struct frame_info_t frame_info;
// 	// 	int n;
// 	// 	memset(rcv_payload, 0, 2000);
// 	// 	n = receive_frame(device_, rcv_payload, 2000, &frame_info, 1);

// 	// 	if (n > 0) {
// 	// 		dump_buffer(rcv_payload, n);

// 	// 		// get current time
// 	// 		struct timeval tv_r;
// 	// 		gettimeofday(&tv_r, 0);

// 	// 		seqno++;
// 	// 		if (frame_info.was_tx) {
// 	// 			printf("                                                                               ");
// 	// 		}

// 	// 		printf("%ld.%06ld: %s packet %5d took %9lld ns with %d retries status %s power %f noise %f\n", tv_r.tv_sec,
// tv_r.tv_usec, (frame_info.was_tx?"tx":"rx"), seqno, frame_info.tx_tsft, frame_info.tx_retries,
// (frame_info.tx_successful?"OK":"ERR"), frame_info.rcv_power, frame_info.noise_power);
// 	// 	}
// 	// }

// 	//ros::Timer send_timer_;
// 	//ros::Timer recv_timer_;
// 	//ros::NodeHandle nh_;
// 	char packet_buffer_[150];
// 	int packet_len_;
// 	device_t* device_;
// 	tTxOpts tx_options_;
// };

int main(int argc, char* argv[])
{
  // Set up ROS.
  // assert(argc>1);
  // std::string name(argv[1]);
  // std::string name("robot1");
  //    printf("Name: %s\n",name.c_str());
  // ros::init(argc, argv, name);

  //	signal(SIGTERM, &txsignalhandler);
  //	signal(SIGINT, &txsignalhandler);
  //	signal(SIGABRT, &txsignalhandler);

  // Build packet

  // Packet built

  // Sender sender(name);

  char packet_buffer_[150];
  int packet_len_;
  device_t* device_;
  tTxOpts tx_options_;

  tMK2MACAddr src_mac = { 0x18, 0xa6, 0xf7, 0x1c, 0x99, 0x90 };
  tMK2MACAddr dst_mac = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
  tx_options_.TxCHOpts.ChannelNumber = 1;
  tx_options_.TxCHOpts.Priority = MK2_PRIO_0;
  tx_options_.TxCHOpts.Service = MK2_QOS_NOACK;
  tx_options_.TxCHOpts.pMCS = MK2MCS_R12QAM16;  // MK2MCS_R12BPSK;
  tx_options_.TxCHOpts.TxPower = 20;
  tx_options_.TxCHOpts.pTxAntenna = MK2_TXANT_DEFAULT;
  tx_options_.TxCHOpts.Bandwidth = MK2BW_20MHz;
  memcpy(tx_options_.TxCHOpts.DestAddr, dst_mac, sizeof tx_options_.TxCHOpts.DestAddr);
  tx_options_.TxCHOpts.EtherType = 0x88b5;
  memcpy(tx_options_.SrcAddr, src_mac, sizeof tx_options_.SrcAddr);
  tx_options_.pInterfaceName = "mon0";

  device_ = (device_t*)open_device(&tx_options_);

  // pthread_t receive_thread_id;
  // pthread_create(&receive_thread_id, NULL, receive_thread, (void *)device);

  // packet_len_ = name.length();
  packet_len_ = 32;
  //		memcpy(packet_buffer_, name.c_str(), packet_len_);
  memcpy(packet_buffer_, "HELLO WORLD --- HELLO WORLD --- HELLO WORLD", 32);

  // while (run) {
  // 	// sleep
  // 	struct timespec time[1];
  // 	time[0].tv_sec = 0;
  // 	time[0].tv_nsec = (10 * 1000* 1000 ); // 500 ms
  // 	nanosleep(time, NULL);

  // 	// send
  // 	printf("sending packet...\n");
  // 	send_frame(device, packet_buffer, packet_len, tx_options.TxCHOpts.TxPower);

  //}
  // ros::spin();
  int i = 0;
  for (i = 0; i < 1000; ++i)
  {
    struct timespec time[1];
    time[0].tv_sec = 0;
    time[0].tv_nsec = (500 * 1000 * 1000);  // 500 ms
    nanosleep(time, NULL);

    // sender.send();
    printf("Sending packet\n");
    send_frame(device_, packet_buffer_, packet_len_, tx_options_.TxCHOpts.TxPower);
  }
  close_device(device_);
  return 0;
}
