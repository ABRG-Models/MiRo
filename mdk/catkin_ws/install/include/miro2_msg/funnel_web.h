// Generated by gencpp from file miro2_msg/funnel_web.msg
// DO NOT EDIT!


#ifndef MIRO2_MSG_MESSAGE_FUNNEL_WEB_H
#define MIRO2_MSG_MESSAGE_FUNNEL_WEB_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace miro2_msg
{
template <class ContainerAllocator>
struct funnel_web_
{
  typedef funnel_web_<ContainerAllocator> Type;

  funnel_web_()
    : cliff()
    , light()
    , sonar(0.0)
    , touch()
    , illum()
    , audio_level(0.0)
    , tone()  {
      cliff.assign(0.0);

      light.assign(0.0);

      touch.assign(0);

      illum.assign(0);

      tone.assign(0);
  }
  funnel_web_(const ContainerAllocator& _alloc)
    : cliff()
    , light()
    , sonar(0.0)
    , touch()
    , illum()
    , audio_level(0.0)
    , tone()  {
  (void)_alloc;
      cliff.assign(0.0);

      light.assign(0.0);

      touch.assign(0);

      illum.assign(0);

      tone.assign(0);
  }



   typedef boost::array<float, 2>  _cliff_type;
  _cliff_type cliff;

   typedef boost::array<float, 4>  _light_type;
  _light_type light;

   typedef float _sonar_type;
  _sonar_type sonar;

   typedef boost::array<uint16_t, 2>  _touch_type;
  _touch_type touch;

   typedef boost::array<uint32_t, 6>  _illum_type;
  _illum_type illum;

   typedef float _audio_level_type;
  _audio_level_type audio_level;

   typedef boost::array<uint16_t, 3>  _tone_type;
  _tone_type tone;





  typedef boost::shared_ptr< ::miro2_msg::funnel_web_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::miro2_msg::funnel_web_<ContainerAllocator> const> ConstPtr;

}; // struct funnel_web_

typedef ::miro2_msg::funnel_web_<std::allocator<void> > funnel_web;

typedef boost::shared_ptr< ::miro2_msg::funnel_web > funnel_webPtr;
typedef boost::shared_ptr< ::miro2_msg::funnel_web const> funnel_webConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::miro2_msg::funnel_web_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::miro2_msg::funnel_web_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::miro2_msg::funnel_web_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::miro2_msg::funnel_web_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::miro2_msg::funnel_web_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::miro2_msg::funnel_web_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::miro2_msg::funnel_web_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::miro2_msg::funnel_web_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::miro2_msg::funnel_web_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d76c86fb9327536412ade3373c6dd88e";
  }

  static const char* value(const ::miro2_msg::funnel_web_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd76c86fb93275364ULL;
  static const uint64_t static_value2 = 0x12ade3373c6dd88eULL;
};

template<class ContainerAllocator>
struct DataType< ::miro2_msg::funnel_web_<ContainerAllocator> >
{
  static const char* value()
  {
    return "miro2_msg/funnel_web";
  }

  static const char* value(const ::miro2_msg::funnel_web_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::miro2_msg::funnel_web_<ContainerAllocator> >
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
#\n\
#	This message packages several messages for simple\n\
#	delivery to the web client.\n\
\n\
\n\
\n\
float32[2] cliff\n\
float32[4] light\n\
float32 sonar\n\
uint16[2] touch\n\
uint32[6] illum\n\
float32 audio_level\n\
uint16[3] tone\n\
\n\
\n\
\n\
\n\
";
  }

  static const char* value(const ::miro2_msg::funnel_web_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::miro2_msg::funnel_web_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.cliff);
      stream.next(m.light);
      stream.next(m.sonar);
      stream.next(m.touch);
      stream.next(m.illum);
      stream.next(m.audio_level);
      stream.next(m.tone);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct funnel_web_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::miro2_msg::funnel_web_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::miro2_msg::funnel_web_<ContainerAllocator>& v)
  {
    s << indent << "cliff[]" << std::endl;
    for (size_t i = 0; i < v.cliff.size(); ++i)
    {
      s << indent << "  cliff[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.cliff[i]);
    }
    s << indent << "light[]" << std::endl;
    for (size_t i = 0; i < v.light.size(); ++i)
    {
      s << indent << "  light[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.light[i]);
    }
    s << indent << "sonar: ";
    Printer<float>::stream(s, indent + "  ", v.sonar);
    s << indent << "touch[]" << std::endl;
    for (size_t i = 0; i < v.touch.size(); ++i)
    {
      s << indent << "  touch[" << i << "]: ";
      Printer<uint16_t>::stream(s, indent + "  ", v.touch[i]);
    }
    s << indent << "illum[]" << std::endl;
    for (size_t i = 0; i < v.illum.size(); ++i)
    {
      s << indent << "  illum[" << i << "]: ";
      Printer<uint32_t>::stream(s, indent + "  ", v.illum[i]);
    }
    s << indent << "audio_level: ";
    Printer<float>::stream(s, indent + "  ", v.audio_level);
    s << indent << "tone[]" << std::endl;
    for (size_t i = 0; i < v.tone.size(); ++i)
    {
      s << indent << "  tone[" << i << "]: ";
      Printer<uint16_t>::stream(s, indent + "  ", v.tone[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // MIRO2_MSG_MESSAGE_FUNNEL_WEB_H
