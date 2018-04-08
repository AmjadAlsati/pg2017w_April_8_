/*
 * util.c
 *
 *  Created on: May 13, 2013
 *      Author: Michele Segata <segata@ccs-labs.org>
 */

#include "cooperative_driving/networking/util.h"

#include <math.h>

#include <net/if.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

double get_gps_distance(double lat1, double lon1, double lat2, double lon2)
{
  double R = 6371;                     // Radius of the earth in km
  double dLat = deg2rad(lat2 - lat1);  // deg2rad below
  double dLon = deg2rad(lon2 - lon1);
  double a = sin(dLat / 2) * sin(dLat / 2) + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  double d = R * c;  // Distance in km
  return d * 1000;   // distance in m
}

double deg2rad(double deg)
{
  return deg * (M_PI / 180.0);
}

void get_my_mac(unsigned char* hw_mac_addr)
{
  int s;
  struct ifreq buffer;

  s = socket(PF_INET, SOCK_DGRAM, 0);

  memset(&buffer, 0x00, sizeof(buffer));

  strcpy(buffer.ifr_name, "mon0");
  ioctl(s, SIOCGIFHWADDR, &buffer);

  close(s);

  memcpy(hw_mac_addr, buffer.ifr_hwaddr.sa_data, 6);
}

void get_readable_buffer(char* buf, int len, char* out)
{
  int i;
  for (i = 0; i < len; ++i)
  {
    out[i] = ((buf[i] < '!') ? '.' : buf[i]);
  }
  out[len] = '\0';
}

long get_sequence_number(char* msg)
{
  char* first_semicolon = strchr(msg, ';');

  return strtol(msg, &first_semicolon, 10);
}

enum MessageType get_message_type(char* msg)
{
  char* first_semicolon = strchr(msg, ';');
  char* second_semicolon = strchr(first_semicolon + 1, ';');

  return strtol(first_semicolon + 1, &second_semicolon, 10);
}

int trim_message(char* msg, int msg_len)
{
  char* first_semicolon = strchr(msg, ';');
  char* second_semicolon = strchr(first_semicolon + 1, ';');
  char* third_semicolon = strchr(second_semicolon + 1, ';');

  int substr_len = third_semicolon - msg;

  int newlen = msg_len - substr_len - 1;

  // Cut msg to get string behind the first semicolon (remove sequence number)
  memmove(msg, third_semicolon + 1, newlen + 1);

  return substr_len + 1;
}

void print_bytes(char* msg, int len)
{
  char out_buffer[3 * len + 1];
  out_buffer[3 * len] = 0;
  for (int i = 0; i < len; ++i)
  {
    sprintf(&out_buffer[3 * i], "%02X ", msg[i]);
  }
  printf("Bytes: %s\n", out_buffer);
}