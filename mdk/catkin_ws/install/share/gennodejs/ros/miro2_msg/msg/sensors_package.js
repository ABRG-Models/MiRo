// Auto-generated. Do not edit!

// (in-package miro2_msg.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let nav_msgs = _finder('nav_msgs');
let sensor_msgs = _finder('sensor_msgs');
let std_msgs = _finder('std_msgs');
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class sensors_package {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.battery = null;
      this.cliff = null;
      this.dip = null;
      this.flags = null;
      this.imu_head = null;
      this.imu_body = null;
      this.kinematic_joints = null;
      this.light = null;
      this.odom = null;
      this.sonar = null;
      this.stream = null;
      this.touch_body = null;
      this.touch_head = null;
      this.wheel_speed_cmd = null;
      this.wheel_speed_back_emf = null;
      this.wheel_speed_opto = null;
      this.wheel_effort_pwm = null;
      this.body_pose = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('battery')) {
        this.battery = initObj.battery
      }
      else {
        this.battery = new sensor_msgs.msg.BatteryState();
      }
      if (initObj.hasOwnProperty('cliff')) {
        this.cliff = initObj.cliff
      }
      else {
        this.cliff = new std_msgs.msg.Float32MultiArray();
      }
      if (initObj.hasOwnProperty('dip')) {
        this.dip = initObj.dip
      }
      else {
        this.dip = new std_msgs.msg.UInt16();
      }
      if (initObj.hasOwnProperty('flags')) {
        this.flags = initObj.flags
      }
      else {
        this.flags = new std_msgs.msg.UInt32();
      }
      if (initObj.hasOwnProperty('imu_head')) {
        this.imu_head = initObj.imu_head
      }
      else {
        this.imu_head = new sensor_msgs.msg.Imu();
      }
      if (initObj.hasOwnProperty('imu_body')) {
        this.imu_body = initObj.imu_body
      }
      else {
        this.imu_body = new sensor_msgs.msg.Imu();
      }
      if (initObj.hasOwnProperty('kinematic_joints')) {
        this.kinematic_joints = initObj.kinematic_joints
      }
      else {
        this.kinematic_joints = new sensor_msgs.msg.JointState();
      }
      if (initObj.hasOwnProperty('light')) {
        this.light = initObj.light
      }
      else {
        this.light = new std_msgs.msg.Float32MultiArray();
      }
      if (initObj.hasOwnProperty('odom')) {
        this.odom = initObj.odom
      }
      else {
        this.odom = new nav_msgs.msg.Odometry();
      }
      if (initObj.hasOwnProperty('sonar')) {
        this.sonar = initObj.sonar
      }
      else {
        this.sonar = new sensor_msgs.msg.Range();
      }
      if (initObj.hasOwnProperty('stream')) {
        this.stream = initObj.stream
      }
      else {
        this.stream = new std_msgs.msg.UInt16MultiArray();
      }
      if (initObj.hasOwnProperty('touch_body')) {
        this.touch_body = initObj.touch_body
      }
      else {
        this.touch_body = new std_msgs.msg.UInt16();
      }
      if (initObj.hasOwnProperty('touch_head')) {
        this.touch_head = initObj.touch_head
      }
      else {
        this.touch_head = new std_msgs.msg.UInt16();
      }
      if (initObj.hasOwnProperty('wheel_speed_cmd')) {
        this.wheel_speed_cmd = initObj.wheel_speed_cmd
      }
      else {
        this.wheel_speed_cmd = new std_msgs.msg.Float32MultiArray();
      }
      if (initObj.hasOwnProperty('wheel_speed_back_emf')) {
        this.wheel_speed_back_emf = initObj.wheel_speed_back_emf
      }
      else {
        this.wheel_speed_back_emf = new std_msgs.msg.Float32MultiArray();
      }
      if (initObj.hasOwnProperty('wheel_speed_opto')) {
        this.wheel_speed_opto = initObj.wheel_speed_opto
      }
      else {
        this.wheel_speed_opto = new std_msgs.msg.Float32MultiArray();
      }
      if (initObj.hasOwnProperty('wheel_effort_pwm')) {
        this.wheel_effort_pwm = initObj.wheel_effort_pwm
      }
      else {
        this.wheel_effort_pwm = new std_msgs.msg.Float32MultiArray();
      }
      if (initObj.hasOwnProperty('body_pose')) {
        this.body_pose = initObj.body_pose
      }
      else {
        this.body_pose = new geometry_msgs.msg.Pose2D();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type sensors_package
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [battery]
    bufferOffset = sensor_msgs.msg.BatteryState.serialize(obj.battery, buffer, bufferOffset);
    // Serialize message field [cliff]
    bufferOffset = std_msgs.msg.Float32MultiArray.serialize(obj.cliff, buffer, bufferOffset);
    // Serialize message field [dip]
    bufferOffset = std_msgs.msg.UInt16.serialize(obj.dip, buffer, bufferOffset);
    // Serialize message field [flags]
    bufferOffset = std_msgs.msg.UInt32.serialize(obj.flags, buffer, bufferOffset);
    // Serialize message field [imu_head]
    bufferOffset = sensor_msgs.msg.Imu.serialize(obj.imu_head, buffer, bufferOffset);
    // Serialize message field [imu_body]
    bufferOffset = sensor_msgs.msg.Imu.serialize(obj.imu_body, buffer, bufferOffset);
    // Serialize message field [kinematic_joints]
    bufferOffset = sensor_msgs.msg.JointState.serialize(obj.kinematic_joints, buffer, bufferOffset);
    // Serialize message field [light]
    bufferOffset = std_msgs.msg.Float32MultiArray.serialize(obj.light, buffer, bufferOffset);
    // Serialize message field [odom]
    bufferOffset = nav_msgs.msg.Odometry.serialize(obj.odom, buffer, bufferOffset);
    // Serialize message field [sonar]
    bufferOffset = sensor_msgs.msg.Range.serialize(obj.sonar, buffer, bufferOffset);
    // Serialize message field [stream]
    bufferOffset = std_msgs.msg.UInt16MultiArray.serialize(obj.stream, buffer, bufferOffset);
    // Serialize message field [touch_body]
    bufferOffset = std_msgs.msg.UInt16.serialize(obj.touch_body, buffer, bufferOffset);
    // Serialize message field [touch_head]
    bufferOffset = std_msgs.msg.UInt16.serialize(obj.touch_head, buffer, bufferOffset);
    // Serialize message field [wheel_speed_cmd]
    bufferOffset = std_msgs.msg.Float32MultiArray.serialize(obj.wheel_speed_cmd, buffer, bufferOffset);
    // Serialize message field [wheel_speed_back_emf]
    bufferOffset = std_msgs.msg.Float32MultiArray.serialize(obj.wheel_speed_back_emf, buffer, bufferOffset);
    // Serialize message field [wheel_speed_opto]
    bufferOffset = std_msgs.msg.Float32MultiArray.serialize(obj.wheel_speed_opto, buffer, bufferOffset);
    // Serialize message field [wheel_effort_pwm]
    bufferOffset = std_msgs.msg.Float32MultiArray.serialize(obj.wheel_effort_pwm, buffer, bufferOffset);
    // Serialize message field [body_pose]
    bufferOffset = geometry_msgs.msg.Pose2D.serialize(obj.body_pose, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type sensors_package
    let len;
    let data = new sensors_package(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [battery]
    data.battery = sensor_msgs.msg.BatteryState.deserialize(buffer, bufferOffset);
    // Deserialize message field [cliff]
    data.cliff = std_msgs.msg.Float32MultiArray.deserialize(buffer, bufferOffset);
    // Deserialize message field [dip]
    data.dip = std_msgs.msg.UInt16.deserialize(buffer, bufferOffset);
    // Deserialize message field [flags]
    data.flags = std_msgs.msg.UInt32.deserialize(buffer, bufferOffset);
    // Deserialize message field [imu_head]
    data.imu_head = sensor_msgs.msg.Imu.deserialize(buffer, bufferOffset);
    // Deserialize message field [imu_body]
    data.imu_body = sensor_msgs.msg.Imu.deserialize(buffer, bufferOffset);
    // Deserialize message field [kinematic_joints]
    data.kinematic_joints = sensor_msgs.msg.JointState.deserialize(buffer, bufferOffset);
    // Deserialize message field [light]
    data.light = std_msgs.msg.Float32MultiArray.deserialize(buffer, bufferOffset);
    // Deserialize message field [odom]
    data.odom = nav_msgs.msg.Odometry.deserialize(buffer, bufferOffset);
    // Deserialize message field [sonar]
    data.sonar = sensor_msgs.msg.Range.deserialize(buffer, bufferOffset);
    // Deserialize message field [stream]
    data.stream = std_msgs.msg.UInt16MultiArray.deserialize(buffer, bufferOffset);
    // Deserialize message field [touch_body]
    data.touch_body = std_msgs.msg.UInt16.deserialize(buffer, bufferOffset);
    // Deserialize message field [touch_head]
    data.touch_head = std_msgs.msg.UInt16.deserialize(buffer, bufferOffset);
    // Deserialize message field [wheel_speed_cmd]
    data.wheel_speed_cmd = std_msgs.msg.Float32MultiArray.deserialize(buffer, bufferOffset);
    // Deserialize message field [wheel_speed_back_emf]
    data.wheel_speed_back_emf = std_msgs.msg.Float32MultiArray.deserialize(buffer, bufferOffset);
    // Deserialize message field [wheel_speed_opto]
    data.wheel_speed_opto = std_msgs.msg.Float32MultiArray.deserialize(buffer, bufferOffset);
    // Deserialize message field [wheel_effort_pwm]
    data.wheel_effort_pwm = std_msgs.msg.Float32MultiArray.deserialize(buffer, bufferOffset);
    // Deserialize message field [body_pose]
    data.body_pose = geometry_msgs.msg.Pose2D.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += sensor_msgs.msg.BatteryState.getMessageSize(object.battery);
    length += std_msgs.msg.Float32MultiArray.getMessageSize(object.cliff);
    length += sensor_msgs.msg.Imu.getMessageSize(object.imu_head);
    length += sensor_msgs.msg.Imu.getMessageSize(object.imu_body);
    length += sensor_msgs.msg.JointState.getMessageSize(object.kinematic_joints);
    length += std_msgs.msg.Float32MultiArray.getMessageSize(object.light);
    length += nav_msgs.msg.Odometry.getMessageSize(object.odom);
    length += sensor_msgs.msg.Range.getMessageSize(object.sonar);
    length += std_msgs.msg.UInt16MultiArray.getMessageSize(object.stream);
    length += std_msgs.msg.Float32MultiArray.getMessageSize(object.wheel_speed_cmd);
    length += std_msgs.msg.Float32MultiArray.getMessageSize(object.wheel_speed_back_emf);
    length += std_msgs.msg.Float32MultiArray.getMessageSize(object.wheel_speed_opto);
    length += std_msgs.msg.Float32MultiArray.getMessageSize(object.wheel_effort_pwm);
    return length + 34;
  }

  static datatype() {
    // Returns string type for a message object
    return 'miro2_msg/sensors_package';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '429d8257e8e981414c3f64c0a1074b4d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #	@section COPYRIGHT
    #	Copyright (C) 2019 Consequential Robotics Ltd
    #	
    #	@section AUTHOR
    #	Consequential Robotics http://consequentialrobotics.com
    #	
    #	@section LICENSE
    #	For a full copy of the license agreement, see LICENSE in the
    #	MDK root directory.
    #	
    #	Subject to the terms of this Agreement, Consequential
    #	Robotics grants to you a limited, non-exclusive, non-
    #	transferable license, without right to sub-license, to use
    #	MIRO Developer Kit in accordance with this Agreement and any
    #	other written agreement with Consequential Robotics.
    #	Consequential Robotics does not transfer the title of MIRO
    #	Developer Kit to you; the license granted to you is not a
    #	sale. This agreement is a binding legal agreement between
    #	Consequential Robotics and the purchasers or users of MIRO
    #	Developer Kit.
    #	
    #	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY
    #	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
    #	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
    #	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
    #	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
    #	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
    #	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
    #	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
    #
    #	This message packages all the messages in /sensors into one
    #	container so that a subscriber can receive them succinctly,
    #	and in synchrony.
    
    
    
    #### HEADER
    
    # standard header
    std_msgs/Header header
    
    
    
    #### CONTENT
    
    sensor_msgs/BatteryState battery
    std_msgs/Float32MultiArray cliff
    std_msgs/UInt16 dip
    std_msgs/UInt32 flags
    sensor_msgs/Imu imu_head
    sensor_msgs/Imu imu_body
    sensor_msgs/JointState kinematic_joints
    std_msgs/Float32MultiArray light
    nav_msgs/Odometry odom
    sensor_msgs/Range sonar
    std_msgs/UInt16MultiArray stream
    std_msgs/UInt16 touch_body
    std_msgs/UInt16 touch_head
    std_msgs/Float32MultiArray wheel_speed_cmd
    std_msgs/Float32MultiArray wheel_speed_back_emf
    std_msgs/Float32MultiArray wheel_speed_opto
    std_msgs/Float32MultiArray wheel_effort_pwm
    
    # available only in the simulator
    geometry_msgs/Pose2D body_pose
    
    
    
    
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    # 0: no frame
    # 1: global frame
    string frame_id
    
    ================================================================================
    MSG: sensor_msgs/BatteryState
    
    # Constants are chosen to match the enums in the linux kernel
    # defined in include/linux/power_supply.h as of version 3.7
    # The one difference is for style reasons the constants are
    # all uppercase not mixed case.
    
    # Power supply status constants
    uint8 POWER_SUPPLY_STATUS_UNKNOWN = 0
    uint8 POWER_SUPPLY_STATUS_CHARGING = 1
    uint8 POWER_SUPPLY_STATUS_DISCHARGING = 2
    uint8 POWER_SUPPLY_STATUS_NOT_CHARGING = 3
    uint8 POWER_SUPPLY_STATUS_FULL = 4
    
    # Power supply health constants
    uint8 POWER_SUPPLY_HEALTH_UNKNOWN = 0
    uint8 POWER_SUPPLY_HEALTH_GOOD = 1
    uint8 POWER_SUPPLY_HEALTH_OVERHEAT = 2
    uint8 POWER_SUPPLY_HEALTH_DEAD = 3
    uint8 POWER_SUPPLY_HEALTH_OVERVOLTAGE = 4
    uint8 POWER_SUPPLY_HEALTH_UNSPEC_FAILURE = 5
    uint8 POWER_SUPPLY_HEALTH_COLD = 6
    uint8 POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE = 7
    uint8 POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE = 8
    
    # Power supply technology (chemistry) constants
    uint8 POWER_SUPPLY_TECHNOLOGY_UNKNOWN = 0
    uint8 POWER_SUPPLY_TECHNOLOGY_NIMH = 1
    uint8 POWER_SUPPLY_TECHNOLOGY_LION = 2
    uint8 POWER_SUPPLY_TECHNOLOGY_LIPO = 3
    uint8 POWER_SUPPLY_TECHNOLOGY_LIFE = 4
    uint8 POWER_SUPPLY_TECHNOLOGY_NICD = 5
    uint8 POWER_SUPPLY_TECHNOLOGY_LIMN = 6
    
    Header  header
    float32 voltage          # Voltage in Volts (Mandatory)
    float32 current          # Negative when discharging (A)  (If unmeasured NaN)
    float32 charge           # Current charge in Ah  (If unmeasured NaN)
    float32 capacity         # Capacity in Ah (last full capacity)  (If unmeasured NaN)
    float32 design_capacity  # Capacity in Ah (design capacity)  (If unmeasured NaN)
    float32 percentage       # Charge percentage on 0 to 1 range  (If unmeasured NaN)
    uint8   power_supply_status     # The charging status as reported. Values defined above
    uint8   power_supply_health     # The battery health metric. Values defined above
    uint8   power_supply_technology # The battery chemistry. Values defined above
    bool    present          # True if the battery is present
    
    float32[] cell_voltage   # An array of individual cell voltages for each cell in the pack
                             # If individual voltages unknown but number of cells known set each to NaN
    string location          # The location into which the battery is inserted. (slot number or plug)
    string serial_number     # The best approximation of the battery serial number
    
    ================================================================================
    MSG: std_msgs/Float32MultiArray
    # Please look at the MultiArrayLayout message definition for
    # documentation on all multiarrays.
    
    MultiArrayLayout  layout        # specification of data layout
    float32[]         data          # array of data
    
    
    ================================================================================
    MSG: std_msgs/MultiArrayLayout
    # The multiarray declares a generic multi-dimensional array of a
    # particular data type.  Dimensions are ordered from outer most
    # to inner most.
    
    MultiArrayDimension[] dim # Array of dimension properties
    uint32 data_offset        # padding elements at front of data
    
    # Accessors should ALWAYS be written in terms of dimension stride
    # and specified outer-most dimension first.
    # 
    # multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
    #
    # A standard, 3-channel 640x480 image with interleaved color channels
    # would be specified as:
    #
    # dim[0].label  = "height"
    # dim[0].size   = 480
    # dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
    # dim[1].label  = "width"
    # dim[1].size   = 640
    # dim[1].stride = 3*640 = 1920
    # dim[2].label  = "channel"
    # dim[2].size   = 3
    # dim[2].stride = 3
    #
    # multiarray(i,j,k) refers to the ith row, jth column, and kth channel.
    
    ================================================================================
    MSG: std_msgs/MultiArrayDimension
    string label   # label of given dimension
    uint32 size    # size of given dimension (in type units)
    uint32 stride  # stride of given dimension
    ================================================================================
    MSG: std_msgs/UInt16
    uint16 data
    
    ================================================================================
    MSG: std_msgs/UInt32
    uint32 data
    ================================================================================
    MSG: sensor_msgs/Imu
    # This is a message to hold data from an IMU (Inertial Measurement Unit)
    #
    # Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
    #
    # If the covariance of the measurement is known, it should be filled in (if all you know is the 
    # variance of each measurement, e.g. from the datasheet, just put those along the diagonal)
    # A covariance matrix of all zeros will be interpreted as "covariance unknown", and to use the
    # data a covariance will have to be assumed or gotten from some other source
    #
    # If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an orientation 
    # estimate), please set element 0 of the associated covariance matrix to -1
    # If you are interpreting this message, please check for a value of -1 in the first element of each 
    # covariance matrix, and disregard the associated estimate.
    
    Header header
    
    geometry_msgs/Quaternion orientation
    float64[9] orientation_covariance # Row major about x, y, z axes
    
    geometry_msgs/Vector3 angular_velocity
    float64[9] angular_velocity_covariance # Row major about x, y, z axes
    
    geometry_msgs/Vector3 linear_acceleration
    float64[9] linear_acceleration_covariance # Row major x, y z 
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
    float64 x
    float64 y
    float64 z
    ================================================================================
    MSG: sensor_msgs/JointState
    # This is a message that holds data to describe the state of a set of torque controlled joints. 
    #
    # The state of each joint (revolute or prismatic) is defined by:
    #  * the position of the joint (rad or m),
    #  * the velocity of the joint (rad/s or m/s) and 
    #  * the effort that is applied in the joint (Nm or N).
    #
    # Each joint is uniquely identified by its name
    # The header specifies the time at which the joint states were recorded. All the joint states
    # in one message have to be recorded at the same time.
    #
    # This message consists of a multiple arrays, one for each part of the joint state. 
    # The goal is to make each of the fields optional. When e.g. your joints have no
    # effort associated with them, you can leave the effort array empty. 
    #
    # All arrays in this message should have the same size, or be empty.
    # This is the only way to uniquely associate the joint name with the correct
    # states.
    
    
    Header header
    
    string[] name
    float64[] position
    float64[] velocity
    float64[] effort
    
    ================================================================================
    MSG: nav_msgs/Odometry
    # This represents an estimate of a position and velocity in free space.  
    # The pose in this message should be specified in the coordinate frame given by header.frame_id.
    # The twist in this message should be specified in the coordinate frame given by the child_frame_id
    Header header
    string child_frame_id
    geometry_msgs/PoseWithCovariance pose
    geometry_msgs/TwistWithCovariance twist
    
    ================================================================================
    MSG: geometry_msgs/PoseWithCovariance
    # This represents a pose in free space with uncertainty.
    
    Pose pose
    
    # Row-major representation of the 6x6 covariance matrix
    # The orientation parameters use a fixed-axis representation.
    # In order, the parameters are:
    # (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
    float64[36] covariance
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/TwistWithCovariance
    # This expresses velocity in free space with uncertainty.
    
    Twist twist
    
    # Row-major representation of the 6x6 covariance matrix
    # The orientation parameters use a fixed-axis representation.
    # In order, the parameters are:
    # (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
    float64[36] covariance
    
    ================================================================================
    MSG: geometry_msgs/Twist
    # This expresses velocity in free space broken into its linear and angular parts.
    Vector3  linear
    Vector3  angular
    
    ================================================================================
    MSG: sensor_msgs/Range
    # Single range reading from an active ranger that emits energy and reports
    # one range reading that is valid along an arc at the distance measured. 
    # This message is  not appropriate for laser scanners. See the LaserScan
    # message if you are working with a laser scanner.
    
    # This message also can represent a fixed-distance (binary) ranger.  This
    # sensor will have min_range===max_range===distance of detection.
    # These sensors follow REP 117 and will output -Inf if the object is detected
    # and +Inf if the object is outside of the detection range.
    
    Header header           # timestamp in the header is the time the ranger
                            # returned the distance reading
    
    # Radiation type enums
    # If you want a value added to this list, send an email to the ros-users list
    uint8 ULTRASOUND=0
    uint8 INFRARED=1
    
    uint8 radiation_type    # the type of radiation used by the sensor
                            # (sound, IR, etc) [enum]
    
    float32 field_of_view   # the size of the arc that the distance reading is
                            # valid for [rad]
                            # the object causing the range reading may have
                            # been anywhere within -field_of_view/2 and
                            # field_of_view/2 at the measured range. 
                            # 0 angle corresponds to the x-axis of the sensor.
    
    float32 min_range       # minimum range value [m]
    float32 max_range       # maximum range value [m]
                            # Fixed distance rangers require min_range==max_range
    
    float32 range           # range data [m]
                            # (Note: values < range_min or > range_max
                            # should be discarded)
                            # Fixed distance rangers only output -Inf or +Inf.
                            # -Inf represents a detection within fixed distance.
                            # (Detection too close to the sensor to quantify)
                            # +Inf represents no detection within the fixed distance.
                            # (Object out of range)
    ================================================================================
    MSG: std_msgs/UInt16MultiArray
    # Please look at the MultiArrayLayout message definition for
    # documentation on all multiarrays.
    
    MultiArrayLayout  layout        # specification of data layout
    uint16[]            data        # array of data
    
    
    ================================================================================
    MSG: geometry_msgs/Pose2D
    # Deprecated
    # Please use the full 3D pose.
    
    # In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.
    
    # If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.
    
    
    # This expresses a position and orientation on a 2D manifold.
    
    float64 x
    float64 y
    float64 theta
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new sensors_package(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.battery !== undefined) {
      resolved.battery = sensor_msgs.msg.BatteryState.Resolve(msg.battery)
    }
    else {
      resolved.battery = new sensor_msgs.msg.BatteryState()
    }

    if (msg.cliff !== undefined) {
      resolved.cliff = std_msgs.msg.Float32MultiArray.Resolve(msg.cliff)
    }
    else {
      resolved.cliff = new std_msgs.msg.Float32MultiArray()
    }

    if (msg.dip !== undefined) {
      resolved.dip = std_msgs.msg.UInt16.Resolve(msg.dip)
    }
    else {
      resolved.dip = new std_msgs.msg.UInt16()
    }

    if (msg.flags !== undefined) {
      resolved.flags = std_msgs.msg.UInt32.Resolve(msg.flags)
    }
    else {
      resolved.flags = new std_msgs.msg.UInt32()
    }

    if (msg.imu_head !== undefined) {
      resolved.imu_head = sensor_msgs.msg.Imu.Resolve(msg.imu_head)
    }
    else {
      resolved.imu_head = new sensor_msgs.msg.Imu()
    }

    if (msg.imu_body !== undefined) {
      resolved.imu_body = sensor_msgs.msg.Imu.Resolve(msg.imu_body)
    }
    else {
      resolved.imu_body = new sensor_msgs.msg.Imu()
    }

    if (msg.kinematic_joints !== undefined) {
      resolved.kinematic_joints = sensor_msgs.msg.JointState.Resolve(msg.kinematic_joints)
    }
    else {
      resolved.kinematic_joints = new sensor_msgs.msg.JointState()
    }

    if (msg.light !== undefined) {
      resolved.light = std_msgs.msg.Float32MultiArray.Resolve(msg.light)
    }
    else {
      resolved.light = new std_msgs.msg.Float32MultiArray()
    }

    if (msg.odom !== undefined) {
      resolved.odom = nav_msgs.msg.Odometry.Resolve(msg.odom)
    }
    else {
      resolved.odom = new nav_msgs.msg.Odometry()
    }

    if (msg.sonar !== undefined) {
      resolved.sonar = sensor_msgs.msg.Range.Resolve(msg.sonar)
    }
    else {
      resolved.sonar = new sensor_msgs.msg.Range()
    }

    if (msg.stream !== undefined) {
      resolved.stream = std_msgs.msg.UInt16MultiArray.Resolve(msg.stream)
    }
    else {
      resolved.stream = new std_msgs.msg.UInt16MultiArray()
    }

    if (msg.touch_body !== undefined) {
      resolved.touch_body = std_msgs.msg.UInt16.Resolve(msg.touch_body)
    }
    else {
      resolved.touch_body = new std_msgs.msg.UInt16()
    }

    if (msg.touch_head !== undefined) {
      resolved.touch_head = std_msgs.msg.UInt16.Resolve(msg.touch_head)
    }
    else {
      resolved.touch_head = new std_msgs.msg.UInt16()
    }

    if (msg.wheel_speed_cmd !== undefined) {
      resolved.wheel_speed_cmd = std_msgs.msg.Float32MultiArray.Resolve(msg.wheel_speed_cmd)
    }
    else {
      resolved.wheel_speed_cmd = new std_msgs.msg.Float32MultiArray()
    }

    if (msg.wheel_speed_back_emf !== undefined) {
      resolved.wheel_speed_back_emf = std_msgs.msg.Float32MultiArray.Resolve(msg.wheel_speed_back_emf)
    }
    else {
      resolved.wheel_speed_back_emf = new std_msgs.msg.Float32MultiArray()
    }

    if (msg.wheel_speed_opto !== undefined) {
      resolved.wheel_speed_opto = std_msgs.msg.Float32MultiArray.Resolve(msg.wheel_speed_opto)
    }
    else {
      resolved.wheel_speed_opto = new std_msgs.msg.Float32MultiArray()
    }

    if (msg.wheel_effort_pwm !== undefined) {
      resolved.wheel_effort_pwm = std_msgs.msg.Float32MultiArray.Resolve(msg.wheel_effort_pwm)
    }
    else {
      resolved.wheel_effort_pwm = new std_msgs.msg.Float32MultiArray()
    }

    if (msg.body_pose !== undefined) {
      resolved.body_pose = geometry_msgs.msg.Pose2D.Resolve(msg.body_pose)
    }
    else {
      resolved.body_pose = new geometry_msgs.msg.Pose2D()
    }

    return resolved;
    }
};

module.exports = sensors_package;
