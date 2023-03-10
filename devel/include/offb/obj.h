// Generated by gencpp from file offb/obj.msg
// DO NOT EDIT!


#ifndef OFFB_MESSAGE_OBJ_H
#define OFFB_MESSAGE_OBJ_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace offb
{
template <class ContainerAllocator>
struct obj_
{
  typedef obj_<ContainerAllocator> Type;

  obj_()
    : object(false)
    , X_c(0.0)
    , Y_c(0.0)
    , Z_c(0.0)  {
    }
  obj_(const ContainerAllocator& _alloc)
    : object(false)
    , X_c(0.0)
    , Y_c(0.0)
    , Z_c(0.0)  {
  (void)_alloc;
    }



   typedef uint8_t _object_type;
  _object_type object;

   typedef double _X_c_type;
  _X_c_type X_c;

   typedef double _Y_c_type;
  _Y_c_type Y_c;

   typedef double _Z_c_type;
  _Z_c_type Z_c;





  typedef boost::shared_ptr< ::offb::obj_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::offb::obj_<ContainerAllocator> const> ConstPtr;

}; // struct obj_

typedef ::offb::obj_<std::allocator<void> > obj;

typedef boost::shared_ptr< ::offb::obj > objPtr;
typedef boost::shared_ptr< ::offb::obj const> objConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::offb::obj_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::offb::obj_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::offb::obj_<ContainerAllocator1> & lhs, const ::offb::obj_<ContainerAllocator2> & rhs)
{
  return lhs.object == rhs.object &&
    lhs.X_c == rhs.X_c &&
    lhs.Y_c == rhs.Y_c &&
    lhs.Z_c == rhs.Z_c;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::offb::obj_<ContainerAllocator1> & lhs, const ::offb::obj_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace offb

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::offb::obj_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::offb::obj_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::offb::obj_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::offb::obj_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::offb::obj_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::offb::obj_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::offb::obj_<ContainerAllocator> >
{
  static const char* value()
  {
    return "281cc9156c896ee80119925d1b2feeb6";
  }

  static const char* value(const ::offb::obj_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x281cc9156c896ee8ULL;
  static const uint64_t static_value2 = 0x0119925d1b2feeb6ULL;
};

template<class ContainerAllocator>
struct DataType< ::offb::obj_<ContainerAllocator> >
{
  static const char* value()
  {
    return "offb/obj";
  }

  static const char* value(const ::offb::obj_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::offb::obj_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool object\n"
"float64 X_c \n"
"float64 Y_c\n"
"float64 Z_c\n"
"\n"
;
  }

  static const char* value(const ::offb::obj_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::offb::obj_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.object);
      stream.next(m.X_c);
      stream.next(m.Y_c);
      stream.next(m.Z_c);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct obj_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::offb::obj_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::offb::obj_<ContainerAllocator>& v)
  {
    s << indent << "object: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.object);
    s << indent << "X_c: ";
    Printer<double>::stream(s, indent + "  ", v.X_c);
    s << indent << "Y_c: ";
    Printer<double>::stream(s, indent + "  ", v.Y_c);
    s << indent << "Z_c: ";
    Printer<double>::stream(s, indent + "  ", v.Z_c);
  }
};

} // namespace message_operations
} // namespace ros

#endif // OFFB_MESSAGE_OBJ_H
