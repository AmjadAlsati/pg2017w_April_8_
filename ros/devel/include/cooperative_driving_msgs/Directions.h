// Generated by gencpp from file cooperative_driving_msgs/Directions.msg
// DO NOT EDIT!


#ifndef COOPERATIVE_DRIVING_MSGS_MESSAGE_DIRECTIONS_H
#define COOPERATIVE_DRIVING_MSGS_MESSAGE_DIRECTIONS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <cooperative_driving_msgs/Direction.h>

namespace cooperative_driving_msgs
{
template <class ContainerAllocator>
struct Directions_
{
  typedef Directions_<ContainerAllocator> Type;

  Directions_()
    : directions()  {
    }
  Directions_(const ContainerAllocator& _alloc)
    : directions(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::cooperative_driving_msgs::Direction_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::cooperative_driving_msgs::Direction_<ContainerAllocator> >::other >  _directions_type;
  _directions_type directions;




  typedef boost::shared_ptr< ::cooperative_driving_msgs::Directions_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cooperative_driving_msgs::Directions_<ContainerAllocator> const> ConstPtr;

}; // struct Directions_

typedef ::cooperative_driving_msgs::Directions_<std::allocator<void> > Directions;

typedef boost::shared_ptr< ::cooperative_driving_msgs::Directions > DirectionsPtr;
typedef boost::shared_ptr< ::cooperative_driving_msgs::Directions const> DirectionsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cooperative_driving_msgs::Directions_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cooperative_driving_msgs::Directions_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace cooperative_driving_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'cooperative_driving_msgs': ['/home/amjad/Desktop/pg2017w/ros/src/cooperative_driving/cooperative_driving_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::cooperative_driving_msgs::Directions_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cooperative_driving_msgs::Directions_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cooperative_driving_msgs::Directions_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cooperative_driving_msgs::Directions_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cooperative_driving_msgs::Directions_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cooperative_driving_msgs::Directions_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cooperative_driving_msgs::Directions_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f467a95e7546b9935ac1d69c0341e385";
  }

  static const char* value(const ::cooperative_driving_msgs::Directions_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf467a95e7546b993ULL;
  static const uint64_t static_value2 = 0x5ac1d69c0341e385ULL;
};

template<class ContainerAllocator>
struct DataType< ::cooperative_driving_msgs::Directions_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cooperative_driving_msgs/Directions";
  }

  static const char* value(const ::cooperative_driving_msgs::Directions_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cooperative_driving_msgs::Directions_<ContainerAllocator> >
{
  static const char* value()
  {
    return "## List all directions of a detected crossing\n\
\n\
# Array of all directions of a crossing \n\
Direction[] directions\n\
================================================================================\n\
MSG: cooperative_driving_msgs/Direction\n\
## Define a direction of a detected crossing, i.e., a turn that can possibly be taken\n\
\n\
# Possible turn at a crossing ('straight', 'left', or 'right')\n\
string direction\n\
";
  }

  static const char* value(const ::cooperative_driving_msgs::Directions_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cooperative_driving_msgs::Directions_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.directions);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Directions_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cooperative_driving_msgs::Directions_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cooperative_driving_msgs::Directions_<ContainerAllocator>& v)
  {
    s << indent << "directions[]" << std::endl;
    for (size_t i = 0; i < v.directions.size(); ++i)
    {
      s << indent << "  directions[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::cooperative_driving_msgs::Direction_<ContainerAllocator> >::stream(s, indent + "    ", v.directions[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // COOPERATIVE_DRIVING_MSGS_MESSAGE_DIRECTIONS_H