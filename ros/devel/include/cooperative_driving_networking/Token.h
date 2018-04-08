// Generated by gencpp from file cooperative_driving_networking/Token.msg
// DO NOT EDIT!


#ifndef COOPERATIVE_DRIVING_NETWORKING_MESSAGE_TOKEN_H
#define COOPERATIVE_DRIVING_NETWORKING_MESSAGE_TOKEN_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace cooperative_driving_networking
{
template <class ContainerAllocator>
struct Token_
{
  typedef Token_<ContainerAllocator> Type;

  Token_()
    : header()
    , message_id(0)
    , sender_id(0)
    , token_id(0)  {
    }
  Token_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , message_id(0)
    , sender_id(0)
    , token_id(0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint32_t _message_id_type;
  _message_id_type message_id;

   typedef uint8_t _sender_id_type;
  _sender_id_type sender_id;

   typedef uint32_t _token_id_type;
  _token_id_type token_id;




  typedef boost::shared_ptr< ::cooperative_driving_networking::Token_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cooperative_driving_networking::Token_<ContainerAllocator> const> ConstPtr;

}; // struct Token_

typedef ::cooperative_driving_networking::Token_<std::allocator<void> > Token;

typedef boost::shared_ptr< ::cooperative_driving_networking::Token > TokenPtr;
typedef boost::shared_ptr< ::cooperative_driving_networking::Token const> TokenConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cooperative_driving_networking::Token_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cooperative_driving_networking::Token_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace cooperative_driving_networking

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'cooperative_driving_networking': ['/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::cooperative_driving_networking::Token_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cooperative_driving_networking::Token_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cooperative_driving_networking::Token_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cooperative_driving_networking::Token_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cooperative_driving_networking::Token_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cooperative_driving_networking::Token_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cooperative_driving_networking::Token_<ContainerAllocator> >
{
  static const char* value()
  {
    return "2cf72de3eab0cac539f4018d2c17b401";
  }

  static const char* value(const ::cooperative_driving_networking::Token_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x2cf72de3eab0cac5ULL;
  static const uint64_t static_value2 = 0x39f4018d2c17b401ULL;
};

template<class ContainerAllocator>
struct DataType< ::cooperative_driving_networking::Token_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cooperative_driving_networking/Token";
  }

  static const char* value(const ::cooperative_driving_networking::Token_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cooperative_driving_networking::Token_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Used in the demo application for the token passing protocol\n\
\n\
# Common message header\n\
# Standard ROS message header\n\
Header header\n\
# Applicationwise uniquie identifier of this message content\n\
uint32 message_id\n\
# The id of the robot which sends this message\n\
uint8 sender_id\n\
# Specific data of this message type\n\
# The token forwared by this messages represented via its id\n\
uint32 token_id\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
";
  }

  static const char* value(const ::cooperative_driving_networking::Token_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cooperative_driving_networking::Token_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.message_id);
      stream.next(m.sender_id);
      stream.next(m.token_id);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Token_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cooperative_driving_networking::Token_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cooperative_driving_networking::Token_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "message_id: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.message_id);
    s << indent << "sender_id: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.sender_id);
    s << indent << "token_id: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.token_id);
  }
};

} // namespace message_operations
} // namespace ros

#endif // COOPERATIVE_DRIVING_NETWORKING_MESSAGE_TOKEN_H
