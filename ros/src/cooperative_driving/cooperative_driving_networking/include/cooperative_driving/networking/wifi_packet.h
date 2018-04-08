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
#pragma once

#include <string>
#include <memory>

#include "message_types.h"

namespace cooperative_driving
{
/** Class defining the structure of  a packet's header.
 * 
 * Each packet consists of a header (some fields) and the payload (some bytes).
 * The structure of the header depends on the elements in attribute Header::fields_ .
 * Consequently, the size of a data field (e.g. type and sequence number) depends on
 * the data type chosen for code representation (e.g. seuence number: long ==> 4 Bytes).
 * 
 * To extend the header, the following steps must be performed:
 * 1. Add a field to the type DataFields
 * 2. Add a setter for the field
 * 3. Extend the implementation of method Header::as_string (optional but recommended)
 * 4. Extend implementation of WiFiPacket so that the field is sensibly used (if desired).
 */
class Header
{
public:
  /** Standard constructor
   *
   */
  Header();
  
  /** Constructor initializing data fields.
   *
   * This constructor can be used if a packet is received as a single byte string.
   * It assumes, that the header has been packed by Header::pack before, otherwise the behaviour is undefined.
   * 
   * \param [in]     data  Byte string containing the header. Header data must lie within the first bytes the byte string.
   */
  Header(uint8_t *data);

  /** Returns header length.
   *
   * \return Number of bytes (equal to size of DataFields).
   */
  size_t get_length();

  /** Returns byte reprsentation of the header data.
   *
   * \param[in] length  Size of returned byte string
   * 
   * \return Attribute Header::fields_ as array of bytes.
   */
  const uint8_t* serialize();

  /** Sets message type.
   *
   * The message type is one of those defined in enum MessageType.
   * 
   * \param[in]  type  The message type
   */
  void set_type(MessageType type);

  /** Gets the message type.
   *
   * The message type is one of those defined in enum MessageType.
   * 
   * \return     The message type.
   */
  enum MessageType get_type();

  /** Sets the sequence number.
   *
   * \param[in]  seqno  The sequence number.
   */
  void set_sequence_number(long seqno);
  
  /** Returns the sequence number.
   *
   * \return The sequence number.
   */
  long get_sequence_number();

  /** Return header as readable string.
   *
   * \return  String representation of attribute Header::fields_.
   */
  std::string as_string();

private:

  /** Structure of the header.
   * 
   * The order in which the fields are defined is irrelevant 
   * since it is converted to a byte array according to its internal memory layout.
   */
  struct DataFields
  {
    MessageType type_ = UNKNOWN;
    long sequence_number_;
  };

  DataFields fields_; //!< The data fields of the header.
  std::unique_ptr<uint8_t> packed_header_; //!< The header in binary format. This is meant for temporary use and should not be used outside of method Header::pack.
};


/** Class for handling packets that are injected into a WiFi channel.
 * 
 * This class contains two main components: header and payload.
 * 
 * The header is an object of type Header while the payload is an array of bytes.
 * 
 * A byte representation of the header can be obtained by calling Header::pack.
 * To get a binary representation of the whole packet, one must call method WiFiPacket::pack.
 */
class WiFiPacket 
{
public:
  /** Constructor, initializing the payload with the passed string.
   *
   * \param[in]  content  The content which is saved binary as payload.
   */
  WiFiPacket(const std::string &content);

  /** Constructor, initializing payload and header according to passed data.
   *
   * This constructor can be used if a packet is received as as single byte string.
   * It is assumed, that data has the following structure: 
   * ```
   * ----------------------------
   * |header bytes|payload bytes|
   * ----------------------------
   * ```
   * The first part is then passed to the class Header and the second part is saved in attribute WiFiPacket::payload_.
   * 
   * \param[in]  data     The binary data.
   * \param[in]  n_bytes  Length of the byte string (number of bytes).
   */
  WiFiPacket(uint8_t* data, size_t n_bytes);
  
  /** Standard constructor.
   */
  WiFiPacket();


  /** Sets the payload.
   *
   * Parameter data is first converted into bytes and then copied into WiFiPacket::payload_. 
   * \param[in]  data  String that shall be saved as payload.
   */
  void set_payload(const std::string &data);
  
  /** Sets the payload as raw data.
   *
   * \param[in]  data     The binary data.
   * \param[in]  n_bytes  Length of the byte string (number of bytes).
   */
  void set_payload(const uint8_t *data, size_t n_bytes);

  /** Convert packet into byte array.
   *
   * This method concatenates the binary representation of the header and the payload.
   * 
   * \return Array containing packet as raw bytes.
   */
  const uint8_t* serialize(size_t &length);

  /** Returns the payload.
   *
   * \return The payload.
   */
  const uint8_t* get_payload();
  
  /** Returns the payload length in number of bytes.
   *
   * \return  The payload length in number of bytes.
   */
  size_t get_payload_length();
  
  /** Returns total packet length.
   *
   * The result is the sum of the length of the header's binary representation and the number of payload bytes.
   * 
   * \return     The packet length in number of bytes.
   */
  size_t get_length();

  /** Get pointer to header.
   *
   * \return Pointer to header.
   */
  Header *get_header();

private:
  std::unique_ptr<Header> header_; //!< Pointer to header.
  std::unique_ptr<uint8_t> payload_; //!< Payload as array of bytes.
  size_t payload_len_ = 0; //!< Number of payload bytes.

  std::unique_ptr<uint8_t> packed_packet_; //!< The packet in binary format. This is meant for temporary use and should not be used outside of method WiFiPacket::pack.
};
}  // namespace cooperative_driving
