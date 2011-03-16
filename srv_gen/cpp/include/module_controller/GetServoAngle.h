/* Auto-generated by genmsg_cpp for file /home/jason/ros_packages/module_controller/srv/GetServoAngle.srv */
#ifndef MODULE_CONTROLLER_SERVICE_GETSERVOANGLE_H
#define MODULE_CONTROLLER_SERVICE_GETSERVOANGLE_H
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
struct GetServoAngleRequest_ : public ros::Message
{
  typedef GetServoAngleRequest_<ContainerAllocator> Type;

  GetServoAngleRequest_()
  : ID(0)
  {
  }

  GetServoAngleRequest_(const ContainerAllocator& _alloc)
  : ID(0)
  {
  }

  typedef int8_t _ID_type;
  int8_t ID;


private:
  static const char* __s_getDataType_() { return "module_controller/GetServoAngleRequest"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "0cb5143ed23a5de01874507c8711c4d5"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getServerMD5Sum_() { return "3b3d703516b54eab670621d84fbecd83"; }
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

  typedef boost::shared_ptr< ::module_controller::GetServoAngleRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::module_controller::GetServoAngleRequest_<ContainerAllocator>  const> ConstPtr;
}; // struct GetServoAngleRequest
typedef  ::module_controller::GetServoAngleRequest_<std::allocator<void> > GetServoAngleRequest;

typedef boost::shared_ptr< ::module_controller::GetServoAngleRequest> GetServoAngleRequestPtr;
typedef boost::shared_ptr< ::module_controller::GetServoAngleRequest const> GetServoAngleRequestConstPtr;


template <class ContainerAllocator>
struct GetServoAngleResponse_ : public ros::Message
{
  typedef GetServoAngleResponse_<ContainerAllocator> Type;

  GetServoAngleResponse_()
  : Angle(0.0)
  {
  }

  GetServoAngleResponse_(const ContainerAllocator& _alloc)
  : Angle(0.0)
  {
  }

  typedef float _Angle_type;
  float Angle;


private:
  static const char* __s_getDataType_() { return "module_controller/GetServoAngleResponse"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "3c12d5a3eed4d7589fde4b3bcd93aff0"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getServerMD5Sum_() { return "3b3d703516b54eab670621d84fbecd83"; }
public:
  ROS_DEPRECATED static const std::string __s_getServerMD5Sum() { return __s_getServerMD5Sum_(); }

  ROS_DEPRECATED const std::string __getServerMD5Sum() const { return __s_getServerMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "float32 Angle\n\
\n\
\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, Angle);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, Angle);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(Angle);
    return size;
  }

  typedef boost::shared_ptr< ::module_controller::GetServoAngleResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::module_controller::GetServoAngleResponse_<ContainerAllocator>  const> ConstPtr;
}; // struct GetServoAngleResponse
typedef  ::module_controller::GetServoAngleResponse_<std::allocator<void> > GetServoAngleResponse;

typedef boost::shared_ptr< ::module_controller::GetServoAngleResponse> GetServoAngleResponsePtr;
typedef boost::shared_ptr< ::module_controller::GetServoAngleResponse const> GetServoAngleResponseConstPtr;

struct GetServoAngle
{

typedef GetServoAngleRequest Request;
typedef GetServoAngleResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct GetServoAngle
} // namespace module_controller

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::module_controller::GetServoAngleRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "0cb5143ed23a5de01874507c8711c4d5";
  }

  static const char* value(const  ::module_controller::GetServoAngleRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x0cb5143ed23a5de0ULL;
  static const uint64_t static_value2 = 0x1874507c8711c4d5ULL;
};

template<class ContainerAllocator>
struct DataType< ::module_controller::GetServoAngleRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "module_controller/GetServoAngleRequest";
  }

  static const char* value(const  ::module_controller::GetServoAngleRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::module_controller::GetServoAngleRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int8 ID\n\
\n\
";
  }

  static const char* value(const  ::module_controller::GetServoAngleRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::module_controller::GetServoAngleRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::module_controller::GetServoAngleResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "3c12d5a3eed4d7589fde4b3bcd93aff0";
  }

  static const char* value(const  ::module_controller::GetServoAngleResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x3c12d5a3eed4d758ULL;
  static const uint64_t static_value2 = 0x9fde4b3bcd93aff0ULL;
};

template<class ContainerAllocator>
struct DataType< ::module_controller::GetServoAngleResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "module_controller/GetServoAngleResponse";
  }

  static const char* value(const  ::module_controller::GetServoAngleResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::module_controller::GetServoAngleResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float32 Angle\n\
\n\
\n\
";
  }

  static const char* value(const  ::module_controller::GetServoAngleResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::module_controller::GetServoAngleResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::module_controller::GetServoAngleRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.ID);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct GetServoAngleRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::module_controller::GetServoAngleResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.Angle);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct GetServoAngleResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<module_controller::GetServoAngle> {
  static const char* value() 
  {
    return "3b3d703516b54eab670621d84fbecd83";
  }

  static const char* value(const module_controller::GetServoAngle&) { return value(); } 
};

template<>
struct DataType<module_controller::GetServoAngle> {
  static const char* value() 
  {
    return "module_controller/GetServoAngle";
  }

  static const char* value(const module_controller::GetServoAngle&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<module_controller::GetServoAngleRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "3b3d703516b54eab670621d84fbecd83";
  }

  static const char* value(const module_controller::GetServoAngleRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<module_controller::GetServoAngleRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "module_controller/GetServoAngle";
  }

  static const char* value(const module_controller::GetServoAngleRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<module_controller::GetServoAngleResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "3b3d703516b54eab670621d84fbecd83";
  }

  static const char* value(const module_controller::GetServoAngleResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<module_controller::GetServoAngleResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "module_controller/GetServoAngle";
  }

  static const char* value(const module_controller::GetServoAngleResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // MODULE_CONTROLLER_SERVICE_GETSERVOANGLE_H

