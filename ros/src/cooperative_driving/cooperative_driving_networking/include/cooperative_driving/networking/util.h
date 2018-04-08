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
/*
 * util.h
 *
 *  Created on: May 13, 2013
 *      Author: Michele Segata <segata@ccs-labs.org>
 */

#ifndef UTIL_H_
#define UTIL_H_

#include "message_types.h"

/**
 * Return distance between two GPS coordinates, in meters
 *
 * \param lat1 latitude of the first position
 * \param lon1 longitude of the first position
 * \param lat2 latitude of the second position
 * \param lon2 longitude of the second position
 * \return the distance in meters
 */
double get_gps_distance(double lat1, double lon1, double lat2, double lon2);

/**
 * Transforms an angle in degrees to radians
 *
 * \param deg angle in degrees
 * \return angle in radians
 */
double deg2rad(double deg);

/** Get my own MAC address.
 *
 * \param[out]   hw_mac_addr  Buffer in which the MAC address is stored.
 */
void get_my_mac(unsigned char* hw_mac_addr);

/** Makes a buffer more readable.
 *
 * Received buffers can contain plenty of bytes, not all of which are human readable.
 * Thus, this function converts not readable bytes (e.g. ASCII control symbols)
 * into periods (".") and keeps the readable ones (ASCII value >= 33).
 * The function ensures that the  output (parameter `out`) is a nullterminated string.
 *
 * \param[in]  buf   The input buffer
 * \param[in]  len   Number of bytes in buf
 * \param[out] out	 Output buffer containing only readable characters. It is assumed that out has already been
 * allocated
 * sufficient memory (at least n+1).
 */
void get_readable_buffer(char* buf, int len, char* out);

/** Read sequence number from received string.
 *
 * Auxiliary function that finds a sequence number in the given message.
 * It must be placed in front of the first semicolon delimiter.
 *
 * \param[out] msg       Received message in the following format `<seqno>;<name>;<data>`.
 *
 * \return    Sequence number of msg
 */
long get_sequence_number(char* msg);

/** Read message type from received string.
 *
 * Auxiliary function that finds a message type in the given message.
 * It must be placed right after the sequence number (divided by ';').
 *
 * \param[in]  msg   The message
 *
 * \return     The message type.
 * 
 * \todo This function could be moved to a different/separate file (e.g. message_types.h).
 */
enum MessageType get_message_type(char* msg);

/** Extract sender's name and sequence number.
*
* Auxiliary function that removes the first two tokens (sequence number and sender's name) from a passed message.
*
* \param[out] msg       Received message in the following format `<seqno>;<name>;<data>`. After proper termination, msg
* contains only `<data>`
*                       which is the plain payload being received.
* \param[in]  msg_len   Length of msg. Since the data part is a non-null-terminated byte string, this needs to be given
* explicitly.
*
* \return     Number of bytes that were cut off.
*/

int trim_message(char* msg, int msg_len);

/** Print message as series of hexadecimal numbers
 *
 * \param[in]  msg   The printed message
 * \param[in]  len   The message's length
 */
void print_bytes(char* msg, int len);

#endif /* UTIL_H_ */
