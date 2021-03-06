// Generated by gencpp from file cooperative_driving_logic/ChangeState.msg
// DO NOT EDIT!


#ifndef COOPERATIVE_DRIVING_LOGIC_MESSAGE_CHANGESTATE_H
#define COOPERATIVE_DRIVING_LOGIC_MESSAGE_CHANGESTATE_H

#include <ros/service_traits.h>


#include <cooperative_driving_logic/ChangeStateRequest.h>
#include <cooperative_driving_logic/ChangeStateResponse.h>


namespace cooperative_driving_logic
{

struct ChangeState
{

typedef ChangeStateRequest Request;
typedef ChangeStateResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct ChangeState
} // namespace cooperative_driving_logic


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::cooperative_driving_logic::ChangeState > {
  static const char* value()
  {
    return "47b07bdab417ccb3c955c49a6d18057b";
  }

  static const char* value(const ::cooperative_driving_logic::ChangeState&) { return value(); }
};

template<>
struct DataType< ::cooperative_driving_logic::ChangeState > {
  static const char* value()
  {
    return "cooperative_driving_logic/ChangeState";
  }

  static const char* value(const ::cooperative_driving_logic::ChangeState&) { return value(); }
};


// service_traits::MD5Sum< ::cooperative_driving_logic::ChangeStateRequest> should match 
// service_traits::MD5Sum< ::cooperative_driving_logic::ChangeState > 
template<>
struct MD5Sum< ::cooperative_driving_logic::ChangeStateRequest>
{
  static const char* value()
  {
    return MD5Sum< ::cooperative_driving_logic::ChangeState >::value();
  }
  static const char* value(const ::cooperative_driving_logic::ChangeStateRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::cooperative_driving_logic::ChangeStateRequest> should match 
// service_traits::DataType< ::cooperative_driving_logic::ChangeState > 
template<>
struct DataType< ::cooperative_driving_logic::ChangeStateRequest>
{
  static const char* value()
  {
    return DataType< ::cooperative_driving_logic::ChangeState >::value();
  }
  static const char* value(const ::cooperative_driving_logic::ChangeStateRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::cooperative_driving_logic::ChangeStateResponse> should match 
// service_traits::MD5Sum< ::cooperative_driving_logic::ChangeState > 
template<>
struct MD5Sum< ::cooperative_driving_logic::ChangeStateResponse>
{
  static const char* value()
  {
    return MD5Sum< ::cooperative_driving_logic::ChangeState >::value();
  }
  static const char* value(const ::cooperative_driving_logic::ChangeStateResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::cooperative_driving_logic::ChangeStateResponse> should match 
// service_traits::DataType< ::cooperative_driving_logic::ChangeState > 
template<>
struct DataType< ::cooperative_driving_logic::ChangeStateResponse>
{
  static const char* value()
  {
    return DataType< ::cooperative_driving_logic::ChangeState >::value();
  }
  static const char* value(const ::cooperative_driving_logic::ChangeStateResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // COOPERATIVE_DRIVING_LOGIC_MESSAGE_CHANGESTATE_H
