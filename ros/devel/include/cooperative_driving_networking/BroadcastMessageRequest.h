// Generated by gencpp from file cooperative_driving_networking/BroadcastMessageRequest.msg
// DO NOT EDIT!


#ifndef COOPERATIVE_DRIVING_NETWORKING_MESSAGE_BROADCASTMESSAGEREQUEST_H
#define COOPERATIVE_DRIVING_NETWORKING_MESSAGE_BROADCASTMESSAGEREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace cooperative_driving_networking
{
template <class ContainerAllocator>
struct BroadcastMessageRequest_
{
  typedef BroadcastMessageRequest_<ContainerAllocator> Type;

  BroadcastMessageRequest_()
    : msg()  {
    }
  BroadcastMessageRequest_(const ContainerAllocator& _alloc)
    : msg(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _msg_type;
  _msg_type msg;




  typedef boost::shared_ptr< ::cooperative_driving_networking::BroadcastMessageRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cooperative_driving_networking::BroadcastMessageRequest_<ContainerAllocator> const> ConstPtr;

}; // struct BroadcastMessageRequest_

typedef ::cooperative_driving_networking::BroadcastMessageRequest_<std::allocator<void> > BroadcastMessageRequest;

typedef boost::shared_ptr< ::cooperative_driving_networking::BroadcastMessageRequest > BroadcastMessageRequestPtr;
typedef boost::shared_ptr< ::cooperative_driving_networking::BroadcastMessageRequest const> BroadcastMessageRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cooperative_driving_networking::BroadcastMessageRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cooperative_driving_networking::BroadcastMessageRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace cooperative_driving_networking

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'cooperative_driving_networking': ['/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_networking/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::cooperative_driving_networking::BroadcastMessageRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cooperative_driving_networking::BroadcastMessageRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cooperative_driving_networking::BroadcastMessageRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cooperative_driving_networking::BroadcastMessageRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cooperative_driving_networking::BroadcastMessageRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cooperative_driving_networking::BroadcastMessageRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cooperative_driving_networking::BroadcastMessageRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "7d96ed730776804754140b85e64c862e";
  }

  static const char* value(const ::cooperative_driving_networking::BroadcastMessageRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x7d96ed7307768047ULL;
  static const uint64_t static_value2 = 0x54140b85e64c862eULL;
};

template<class ContainerAllocator>
struct DataType< ::cooperative_driving_networking::BroadcastMessageRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cooperative_driving_networking/BroadcastMessageRequest";
  }

  static const char* value(const ::cooperative_driving_networking::BroadcastMessageRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cooperative_driving_networking::BroadcastMessageRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string msg\n\
";
  }

  static const char* value(const ::cooperative_driving_networking::BroadcastMessageRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cooperative_driving_networking::BroadcastMessageRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.msg);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct BroadcastMessageRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cooperative_driving_networking::BroadcastMessageRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cooperative_driving_networking::BroadcastMessageRequest_<ContainerAllocator>& v)
  {
    s << indent << "msg: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.msg);
  }
};

} // namespace message_operations
} // namespace ros

#endif // COOPERATIVE_DRIVING_NETWORKING_MESSAGE_BROADCASTMESSAGEREQUEST_H
