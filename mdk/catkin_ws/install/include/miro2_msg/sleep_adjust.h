// Generated by gencpp from file miro2_msg/sleep_adjust.msg
// DO NOT EDIT!


#ifndef MIRO2_MSG_MESSAGE_SLEEP_ADJUST_H
#define MIRO2_MSG_MESSAGE_SLEEP_ADJUST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <miro2_msg/adjust.h>
#include <miro2_msg/adjust.h>

namespace miro2_msg
{
template <class ContainerAllocator>
struct sleep_adjust_
{
  typedef sleep_adjust_<ContainerAllocator> Type;

  sleep_adjust_()
    : wakefulness()
    , pressure()  {
    }
  sleep_adjust_(const ContainerAllocator& _alloc)
    : wakefulness(_alloc)
    , pressure(_alloc)  {
  (void)_alloc;
    }



   typedef  ::miro2_msg::adjust_<ContainerAllocator>  _wakefulness_type;
  _wakefulness_type wakefulness;

   typedef  ::miro2_msg::adjust_<ContainerAllocator>  _pressure_type;
  _pressure_type pressure;





  typedef boost::shared_ptr< ::miro2_msg::sleep_adjust_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::miro2_msg::sleep_adjust_<ContainerAllocator> const> ConstPtr;

}; // struct sleep_adjust_

typedef ::miro2_msg::sleep_adjust_<std::allocator<void> > sleep_adjust;

typedef boost::shared_ptr< ::miro2_msg::sleep_adjust > sleep_adjustPtr;
typedef boost::shared_ptr< ::miro2_msg::sleep_adjust const> sleep_adjustConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::miro2_msg::sleep_adjust_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::miro2_msg::sleep_adjust_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace miro2_msg

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'nav_msgs': ['/opt/ros/kinetic/share/nav_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg'], 'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'miro2_msg': ['/tmp/miro2/release/191016/mdk/catkin_ws/src/miro2_msg/msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::miro2_msg::sleep_adjust_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::miro2_msg::sleep_adjust_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::miro2_msg::sleep_adjust_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::miro2_msg::sleep_adjust_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::miro2_msg::sleep_adjust_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::miro2_msg::sleep_adjust_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::miro2_msg::sleep_adjust_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d50186fa3ccd96b438a40f38889ec949";
  }

  static const char* value(const ::miro2_msg::sleep_adjust_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd50186fa3ccd96b4ULL;
  static const uint64_t static_value2 = 0x38a40f38889ec949ULL;
};

template<class ContainerAllocator>
struct DataType< ::miro2_msg::sleep_adjust_<ContainerAllocator> >
{
  static const char* value()
  {
    return "miro2_msg/sleep_adjust";
  }

  static const char* value(const ::miro2_msg::sleep_adjust_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::miro2_msg::sleep_adjust_<ContainerAllocator> >
{
  static const char* value()
  {
    return "#	@section COPYRIGHT\n\
#	Copyright (C) 2019 Consequential Robotics Ltd\n\
#	\n\
#	@section AUTHOR\n\
#	Consequential Robotics http://consequentialrobotics.com\n\
#	\n\
#	@section LICENSE\n\
#	For a full copy of the license agreement, and a complete\n\
#	definition of \"The Software\", see LICENSE in the MDK root\n\
#	directory.\n\
#	\n\
#	Subject to the terms of this Agreement, Consequential\n\
#	Robotics grants to you a limited, non-exclusive, non-\n\
#	transferable license, without right to sub-license, to use\n\
#	\"The Software\" in accordance with this Agreement and any\n\
#	other written agreement with Consequential Robotics.\n\
#	Consequential Robotics does not transfer the title of \"The\n\
#	Software\" to you; the license granted to you is not a sale.\n\
#	This agreement is a binding legal agreement between\n\
#	Consequential Robotics and the purchasers or users of \"The\n\
#	Software\".\n\
#	\n\
#	THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY\n\
#	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE\n\
#	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR\n\
#	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS\n\
#	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR\n\
#	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR\n\
#	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE\n\
#	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.\n\
#	\n\
\n\
adjust wakefulness\n\
adjust pressure\n\
\n\
\n\
================================================================================\n\
MSG: miro2_msg/adjust\n\
#	@section COPYRIGHT\n\
#	Copyright (C) 2019 Consequential Robotics Ltd\n\
#	\n\
#	@section AUTHOR\n\
#	Consequential Robotics http://consequentialrobotics.com\n\
#	\n\
#	@section LICENSE\n\
#	For a full copy of the license agreement, and a complete\n\
#	definition of \"The Software\", see LICENSE in the MDK root\n\
#	directory.\n\
#	\n\
#	Subject to the terms of this Agreement, Consequential\n\
#	Robotics grants to you a limited, non-exclusive, non-\n\
#	transferable license, without right to sub-license, to use\n\
#	\"The Software\" in accordance with this Agreement and any\n\
#	other written agreement with Consequential Robotics.\n\
#	Consequential Robotics does not transfer the title of \"The\n\
#	Software\" to you; the license granted to you is not a sale.\n\
#	This agreement is a binding legal agreement between\n\
#	Consequential Robotics and the purchasers or users of \"The\n\
#	Software\".\n\
#	\n\
#	THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY\n\
#	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE\n\
#	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR\n\
#	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS\n\
#	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR\n\
#	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR\n\
#	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE\n\
#	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.\n\
#	\n\
#\n\
#	Adjust message provides a route for directly adjusting\n\
#	a state of the biomimetic model. There are two ways to\n\
#	specify an adjustment, selected independently for each\n\
#	adjustment channel.\n\
#\n\
#	1) Provide a target value in \"data\" and a \"gamma\" value\n\
#	between 0 and 1 to cause the state to approach the target:\n\
#\n\
#	(at 50Hz)\n\
#	state += gamma * (data - state)\n\
#\n\
#	2) Provide a delta value in \"data\" and set \"gamma\"\n\
#	to -1 to indicate this drive mode:\n\
#\n\
#	(at 50Hz)\n\
#	state += data\n\
#\n\
#	Understood values of gamma, therefore, are:\n\
#	   -1 : add \"data\" to state\n\
#	    0 : do nothing\n\
#	  0-1 : move state towards \"data\"\n\
#	    1 : instantly set state to \"data\"\n\
\n\
float32 data\n\
float32 gamma\n\
\n\
";
  }

  static const char* value(const ::miro2_msg::sleep_adjust_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::miro2_msg::sleep_adjust_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.wakefulness);
      stream.next(m.pressure);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct sleep_adjust_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::miro2_msg::sleep_adjust_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::miro2_msg::sleep_adjust_<ContainerAllocator>& v)
  {
    s << indent << "wakefulness: ";
    s << std::endl;
    Printer< ::miro2_msg::adjust_<ContainerAllocator> >::stream(s, indent + "  ", v.wakefulness);
    s << indent << "pressure: ";
    s << std::endl;
    Printer< ::miro2_msg::adjust_<ContainerAllocator> >::stream(s, indent + "  ", v.pressure);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MIRO2_MSG_MESSAGE_SLEEP_ADJUST_H
