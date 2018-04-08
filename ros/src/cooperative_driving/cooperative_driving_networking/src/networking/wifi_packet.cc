#include <sstream>
#include <cstring>

#include "cooperative_driving/networking/wifi_packet.h"
#include "cooperative_driving/networking/util.h"

namespace cooperative_driving
{

Header::Header()
{
}

Header::Header(uint8_t *data)
{
  std::memcpy(&fields_, data, sizeof(DataFields));
}

size_t Header::get_length()
{
  return sizeof(DataFields);
}

void Header::set_type(MessageType type)
{
  fields_.type_ = type;
}

enum MessageType Header::get_type()
{
  return fields_.type_;
}

const uint8_t* Header::serialize()
{
  packed_header_.reset(new uint8_t[get_length()]);

  std::memcpy(packed_header_.get(), &fields_, sizeof(DataFields));

  return packed_header_.get();
}

void Header::set_sequence_number(long seqno)
{
  fields_.sequence_number_ = seqno;
}

long Header::get_sequence_number()
{
  return fields_.sequence_number_;
}

std::string Header::as_string()
{
  std::stringstream ss;
  ss << fields_.sequence_number_ << "|" << fields_.type_;
  return ss.str();
}

WiFiPacket::WiFiPacket(uint8_t* data, size_t n_bytes)
{
  header_.reset(new Header(data));
  size_t hlen = header_->get_length();
  payload_len_ = n_bytes - hlen;
  payload_.reset(new uint8_t[payload_len_]);
  std::memcpy(payload_.get(), data+hlen, payload_len_);
}

WiFiPacket::WiFiPacket()
{
  header_.reset(new Header());
}

WiFiPacket::WiFiPacket(const std::string &content)
{
  header_.reset(new Header());
  set_payload(content);
}

const uint8_t* WiFiPacket::get_payload()
{
  return payload_.get();
}

void WiFiPacket::set_payload(const std::string &data)
{
  set_payload(reinterpret_cast<const uint8_t*>(data.c_str()), data.length());
}

void WiFiPacket::set_payload(const uint8_t *data, size_t n_bytes)
{
  payload_.reset(new uint8_t[n_bytes]);

  std::memcpy(payload_.get(), data, n_bytes);

  payload_len_ = n_bytes;
}

const uint8_t* WiFiPacket::serialize(size_t &length)
{
  packed_packet_.reset(new uint8_t[get_length()]);

  const uint8_t* packed_header = header_->serialize();
  size_t hlen = header_->get_length();

  std::memcpy(packed_packet_.get(), packed_header, hlen);
  std::memcpy(packed_packet_.get()+hlen, payload_.get(), payload_len_);

  length = get_length();

  return packed_packet_.get();
}

Header *WiFiPacket::get_header()
{
  return header_.get();
}

size_t WiFiPacket::get_payload_length()
{
  return payload_len_;
}

size_t WiFiPacket::get_length()
{
  return header_->get_length() + get_payload_length();
}
}