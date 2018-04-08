/****************************************************************************
Classification: U//FOUO
*//**
    @file     RawSocket.c
    @brief    Demonstrates raw packet injection and capture in C
    @author   Andrew Gorczyca
    @date     2012/5/24
****************************************************************************/
 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <sys/socket.h>
#include <linux/if_packet.h>
#include <linux/if_ether.h>
#include <linux/if.h>
#include <sys/ioctl.h>
 
int RawSocket_new(char * device, int protocol);
void RawSocket_injectDemo(int raw, int numPackets);
void RawSocket_sniffDemo(int raw, int numPackets);
 
main(int argc, char **argv)
{
    /* create a raw socket interface */
    int raw = RawSocket_new("mon1",ETH_P_ALL);
 
    /* demonstrate capabilities */
    if (raw != -1)
    {
        RawSocket_injectDemo(raw,200);
//        RawSocket_sniffDemo(raw, 20);
        close(raw);
    }
    else
    {
        printf("Could not open raw socket!\n");
    }
 
    printf("done\n");
    return 0;
}
 
int RawSocket_new(char * device, int protocol)
{
    int rawsock;
    struct sockaddr_ll sll;
    struct ifreq ifr;
 
    if((rawsock = socket(PF_PACKET, SOCK_RAW, htons(protocol)))== -1)
    {
        /* probably a premissions error */
        return -1;
    }
 
    memset(&sll, 0, sizeof(sll));
    memset(&ifr, 0, sizeof(ifr));
     
    /* get interface index  */
    strncpy((char *)ifr.ifr_name, device, IFNAMSIZ);
    if((ioctl(rawsock, SIOCGIFINDEX, &ifr)) == -1)
    {
        return -1;  /* device not found */
    }
 
    /* Bind our raw socket to this interface */
    sll.sll_family = AF_PACKET;
    sll.sll_ifindex = ifr.ifr_ifindex;
    sll.sll_protocol = htons(protocol); 
 
    if((bind(rawsock, (struct sockaddr *)&sll, sizeof(sll)))== -1)
    {
        return -1;  /* bind error */
    }
 
	printf("\nCompleted with RawSocket_new()\n");
    return rawsock;
}
 
void RawSocket_injectDemo(int raw, int numPackets)
{
	printf("\nInside RawSocket_InkectDemo()");
    char pkt[27] = "abcdefghijklmnopqrstuvwxyz";
    while (numPackets --)
    {
        write(raw, pkt, sizeof(pkt));
    }
}
 
void RawSocket_sniffDemo(int raw, int numPackets)
{
	printf("\nInside RawSocket_sniffDemo()\n");
    struct sockaddr_ll packet_info;
    int packet_info_size = sizeof(packet_info_size);
    uint8_t packet_buffer[2048];
    ssize_t len;

    while (numPackets)
    {//raw,packet_buffer,2048
        if((len = recvfrom(raw, 
			packet_buffer, 
			2048, 
			0, 
			(struct sockaddr*)&packet_info, 
			&packet_info_size)) == -1)
        {
            return;
        }
        else
        {

            printf("\ngot a packet of length %d\n",(int)len);

            uint16_t u16_i = 0;

            printf ("Received data from ");

            for( u16_i=0; u16_i<sizeof(packet_info.sll_addr)-2; u16_i++ )
            {
                printf ("%02x:", packet_info.sll_addr[u16_i]);
            }

//		printf ("Received data %s\n\n", packet_buffer[2048]);

            numPackets--;
        }
    }
}
