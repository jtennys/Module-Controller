/* Auto-generated by genmsg_cpp for file /home/jason/ros_packages/module_controller/srv/GetServoPower.srv */
#ifndef MODULE_CONTROLLER_SERVICE_GETSERVOPOWER_H
#define MODULE_CONTROLLER_SERVICE_GETSERVOPOWER_H
#include <string>
#include <vector>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/message.h"
#include "ros/time.h"

#include "ros/service_traits.h"




namespace module_controller
{
template <class ContainerAllocator>
struct GetServoPowerRequest_ : public ros::Message
{
  typedef GetServoPowerRequest_<ContainerAllocator> Type;

  GetServoPowerRequest_()
  : ID(0)
  {
  }

  GetServoPowerRequest_(const ContainerAllocator& _alloc)
  : ID(0)
  {
  }

  typedef int8_t _ID_type;
  int8_t ID;


private:
  static const char* __s_getDataType_() { return "module_controller/GetServoPowerRequest"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "0cb5143ed23a5de01874507c8711c4d5"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getServerMD5Sum_() { return "63625bde14a8229e56f4eaf5667c7359"; }
public:
  ROS_DEPRECATED static const std::string __s_getServerMD5Sum() { return __s_getServerMD5Sum_(); }

  ROS_DEPRECATED const std::string __getServerMD5Sum() const { return __s_getServerMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "int8 ID\n\
\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, ID);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, ID);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(ID);
    return size;
  }

  typedef boost::shared_ptr< ::module_controller::GetServoPowerRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::module_controller::GetServoPowerRequest_<ContainerAllocator>  const> ConstPtr;
}; // struct GetServoPowerRequest
typedef  ::module_controller::GetServoPowerRequest_<std::allocator<void> > GetServoPowerRequest;

typedef boost::shared_ptr< ::module_controller::GetServoPowerRequest> GetServoPowerRequestPtr;
typedef boost::shared_ptr< ::module_controller::GetServoPowerRequest const> GetServoPowerRequestConstPtr;


template <class ContainerAllocator>
struct GetServoPowerResponse_ : public ros::Message
{
  typedef GetServoPowerResponse_<ContainerAllocator> Type;

  GetServoPowerResponse_()
  : Power(0)
  {
  }

  GetServoPowerResponse_(const ContainerAllocator& _alloc)
  : Power(0)
  {
  }

  typedef int8_t _Power_type;
  int8_t Power;


private:
  static const char* __s_getDataType_() { return "module_controller/GetServoPowerResponse"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "1d4befc03e6691e2b4a0bfe78761754b"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getServerMD5Sum_() { return "63625bde14a8229e56f4eaf5667c7359"; }
public:
  ROS_DEPRECATED static const std::string __s_getServerMD5Sum() { return __s_getServerMD5Sum_(); }

  ROS_DEPRECATED const std::string __getServerMD5Sum() const { return __s_getServerMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "int8 Power\n\
\n\
\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, Power);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, Power);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(Power);
    return size;
  }

  typedef boost::shared_ptr< ::module_controller::GetServoPowerResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::module_controller::GetServoPowerResponse_<ContainerAllocator>  const> ConstPtr;
}; // struct GetServoPowerResponse
typedef  ::module_controller::GetServoPowerResponse_<std::allocator<void> > GetServoPowerResponse;

typedef boost::shared_ptr< ::module_controller::GetServoPowerResponse> GetServoPowerResponsePtr;
typedef boost::shared_ptr< ::module_controller::GetServoPowerResponse const> GetServoPowerResponseConstPtr;

struct GetServoPower
{

typedef GetServoPowerRequest Request;
typedef GetServoPowerResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct GetServoPower
} // namespace module_controller

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::module_controller::GetServoPowerRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "0cb5143ed23a5de01874507c8711c4d5";
  }

  static const char* value(const  ::module_controller::GetServoPowerRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x0cb5143ed23a5de0ULL;
  static const uint64_t static_value2 = 0x1874507c8711c4d5ULL;
};

template<class ContainerAllocator>
struct DataType< ::module_controller::GetServoPowerRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "module_controller/GetServoPowerRequest";
  }

  static const char* value(const  ::module_controller::GetServoPowerRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::module_controller::GetServoPowerRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int8 ID\n\
\n\
";
  }

  static const char* value(const  ::module_controller::GetServoPowerRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::module_controller::GetServoPowerRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::module_controller::GetServoPowerResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "1d4befc03e6691e2b4a0bfe78761754b";
  }

  static const char* value(const  ::module_controller::GetServoPowerResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x1d4befc03e6691e2ULL;
  static const uint64_t static_value2 = 0xb4a0bfe78761754bULL;
};

template<class ContainerAllocator>
struct DataType< ::module_controller::GetServoPowerResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "module_controller/GetServoPowerResponse";
  }

  static const char* value(const  ::module_controller::GetServoPowerResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::module_controller::GetServoPowerResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int8 Power\n\
\n\
\n\
";
  }

  static const char* value(const  ::module_controller::GetServoPowerResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::module_controller::GetServoPowerResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::module_controller::GetServoPowerRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.ID);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct GetServoPowerRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::module_controller::GetServoPowerResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.Power);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct GetServoPowerResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<module_controller::GetServoPower> {
  static const char* value() 
  {
    return "63625bde14a8229e56f4eaf5667c7359";
  }

  static const char* value(const module_controller::GetServoPower&) { return value(); } 
};

template<>
struct DataType<module_controller::GetServoPower> {
  static const char* value() 
  {
    return "module_controller/GetServoPower";
  }

  static const char* value(const module_controller::GetServoPower&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<module_controller::GetServoPowerRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "63625bde14a8229e56f4eaf5667c7359";
  }

  static const char* value(const module_controller::GetServoPowerRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<module_controller::GetServoPowerRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "module_controller/GetServoPower";
  }

  static const char* value(const module_controller::GetServoPowerRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<module_controller::GetServoPowerResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "63625bde14a8229e56f4eaf5667c7359";
  }

  static const char* value(const module_controller::GetServoPowerResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<module_controller::GetServoPowerResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "module_controller/GetServoPower";
  }

  static const char* value(const module_controller::GetServoPowerResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // MODULE_CONTROLLER_SERVICE_GETSERVOPOWER_H
