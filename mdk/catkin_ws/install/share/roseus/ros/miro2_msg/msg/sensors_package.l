;; Auto-generated. Do not edit!


(when (boundp 'miro2_msg::sensors_package)
  (if (not (find-package "MIRO2_MSG"))
    (make-package "MIRO2_MSG"))
  (shadow 'sensors_package (find-package "MIRO2_MSG")))
(unless (find-package "MIRO2_MSG::SENSORS_PACKAGE")
  (make-package "MIRO2_MSG::SENSORS_PACKAGE"))

(in-package "ROS")
;;//! \htmlinclude sensors_package.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "NAV_MSGS"))
  (ros::roseus-add-msgs "nav_msgs"))
(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass miro2_msg::sensors_package
  :super ros::object
  :slots (_header _battery _cliff _dip _flags _imu_head _imu_body _kinematic_joints _light _odom _sonar _stream _touch_body _touch_head _wheel_speed_cmd _wheel_speed_back_emf _wheel_speed_opto _wheel_effort_pwm _body_pose ))

(defmethod miro2_msg::sensors_package
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:battery __battery) (instance sensor_msgs::BatteryState :init))
    ((:cliff __cliff) (instance std_msgs::Float32MultiArray :init))
    ((:dip __dip) (instance std_msgs::UInt16 :init))
    ((:flags __flags) (instance std_msgs::UInt32 :init))
    ((:imu_head __imu_head) (instance sensor_msgs::Imu :init))
    ((:imu_body __imu_body) (instance sensor_msgs::Imu :init))
    ((:kinematic_joints __kinematic_joints) (instance sensor_msgs::JointState :init))
    ((:light __light) (instance std_msgs::Float32MultiArray :init))
    ((:odom __odom) (instance nav_msgs::Odometry :init))
    ((:sonar __sonar) (instance sensor_msgs::Range :init))
    ((:stream __stream) (instance std_msgs::UInt16MultiArray :init))
    ((:touch_body __touch_body) (instance std_msgs::UInt16 :init))
    ((:touch_head __touch_head) (instance std_msgs::UInt16 :init))
    ((:wheel_speed_cmd __wheel_speed_cmd) (instance std_msgs::Float32MultiArray :init))
    ((:wheel_speed_back_emf __wheel_speed_back_emf) (instance std_msgs::Float32MultiArray :init))
    ((:wheel_speed_opto __wheel_speed_opto) (instance std_msgs::Float32MultiArray :init))
    ((:wheel_effort_pwm __wheel_effort_pwm) (instance std_msgs::Float32MultiArray :init))
    ((:body_pose __body_pose) (instance geometry_msgs::Pose2D :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _battery __battery)
   (setq _cliff __cliff)
   (setq _dip __dip)
   (setq _flags __flags)
   (setq _imu_head __imu_head)
   (setq _imu_body __imu_body)
   (setq _kinematic_joints __kinematic_joints)
   (setq _light __light)
   (setq _odom __odom)
   (setq _sonar __sonar)
   (setq _stream __stream)
   (setq _touch_body __touch_body)
   (setq _touch_head __touch_head)
   (setq _wheel_speed_cmd __wheel_speed_cmd)
   (setq _wheel_speed_back_emf __wheel_speed_back_emf)
   (setq _wheel_speed_opto __wheel_speed_opto)
   (setq _wheel_effort_pwm __wheel_effort_pwm)
   (setq _body_pose __body_pose)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:battery
   (&rest __battery)
   (if (keywordp (car __battery))
       (send* _battery __battery)
     (progn
       (if __battery (setq _battery (car __battery)))
       _battery)))
  (:cliff
   (&rest __cliff)
   (if (keywordp (car __cliff))
       (send* _cliff __cliff)
     (progn
       (if __cliff (setq _cliff (car __cliff)))
       _cliff)))
  (:dip
   (&rest __dip)
   (if (keywordp (car __dip))
       (send* _dip __dip)
     (progn
       (if __dip (setq _dip (car __dip)))
       _dip)))
  (:flags
   (&rest __flags)
   (if (keywordp (car __flags))
       (send* _flags __flags)
     (progn
       (if __flags (setq _flags (car __flags)))
       _flags)))
  (:imu_head
   (&rest __imu_head)
   (if (keywordp (car __imu_head))
       (send* _imu_head __imu_head)
     (progn
       (if __imu_head (setq _imu_head (car __imu_head)))
       _imu_head)))
  (:imu_body
   (&rest __imu_body)
   (if (keywordp (car __imu_body))
       (send* _imu_body __imu_body)
     (progn
       (if __imu_body (setq _imu_body (car __imu_body)))
       _imu_body)))
  (:kinematic_joints
   (&rest __kinematic_joints)
   (if (keywordp (car __kinematic_joints))
       (send* _kinematic_joints __kinematic_joints)
     (progn
       (if __kinematic_joints (setq _kinematic_joints (car __kinematic_joints)))
       _kinematic_joints)))
  (:light
   (&rest __light)
   (if (keywordp (car __light))
       (send* _light __light)
     (progn
       (if __light (setq _light (car __light)))
       _light)))
  (:odom
   (&rest __odom)
   (if (keywordp (car __odom))
       (send* _odom __odom)
     (progn
       (if __odom (setq _odom (car __odom)))
       _odom)))
  (:sonar
   (&rest __sonar)
   (if (keywordp (car __sonar))
       (send* _sonar __sonar)
     (progn
       (if __sonar (setq _sonar (car __sonar)))
       _sonar)))
  (:stream
   (&rest __stream)
   (if (keywordp (car __stream))
       (send* _stream __stream)
     (progn
       (if __stream (setq _stream (car __stream)))
       _stream)))
  (:touch_body
   (&rest __touch_body)
   (if (keywordp (car __touch_body))
       (send* _touch_body __touch_body)
     (progn
       (if __touch_body (setq _touch_body (car __touch_body)))
       _touch_body)))
  (:touch_head
   (&rest __touch_head)
   (if (keywordp (car __touch_head))
       (send* _touch_head __touch_head)
     (progn
       (if __touch_head (setq _touch_head (car __touch_head)))
       _touch_head)))
  (:wheel_speed_cmd
   (&rest __wheel_speed_cmd)
   (if (keywordp (car __wheel_speed_cmd))
       (send* _wheel_speed_cmd __wheel_speed_cmd)
     (progn
       (if __wheel_speed_cmd (setq _wheel_speed_cmd (car __wheel_speed_cmd)))
       _wheel_speed_cmd)))
  (:wheel_speed_back_emf
   (&rest __wheel_speed_back_emf)
   (if (keywordp (car __wheel_speed_back_emf))
       (send* _wheel_speed_back_emf __wheel_speed_back_emf)
     (progn
       (if __wheel_speed_back_emf (setq _wheel_speed_back_emf (car __wheel_speed_back_emf)))
       _wheel_speed_back_emf)))
  (:wheel_speed_opto
   (&rest __wheel_speed_opto)
   (if (keywordp (car __wheel_speed_opto))
       (send* _wheel_speed_opto __wheel_speed_opto)
     (progn
       (if __wheel_speed_opto (setq _wheel_speed_opto (car __wheel_speed_opto)))
       _wheel_speed_opto)))
  (:wheel_effort_pwm
   (&rest __wheel_effort_pwm)
   (if (keywordp (car __wheel_effort_pwm))
       (send* _wheel_effort_pwm __wheel_effort_pwm)
     (progn
       (if __wheel_effort_pwm (setq _wheel_effort_pwm (car __wheel_effort_pwm)))
       _wheel_effort_pwm)))
  (:body_pose
   (&rest __body_pose)
   (if (keywordp (car __body_pose))
       (send* _body_pose __body_pose)
     (progn
       (if __body_pose (setq _body_pose (car __body_pose)))
       _body_pose)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; sensor_msgs/BatteryState _battery
    (send _battery :serialization-length)
    ;; std_msgs/Float32MultiArray _cliff
    (send _cliff :serialization-length)
    ;; std_msgs/UInt16 _dip
    (send _dip :serialization-length)
    ;; std_msgs/UInt32 _flags
    (send _flags :serialization-length)
    ;; sensor_msgs/Imu _imu_head
    (send _imu_head :serialization-length)
    ;; sensor_msgs/Imu _imu_body
    (send _imu_body :serialization-length)
    ;; sensor_msgs/JointState _kinematic_joints
    (send _kinematic_joints :serialization-length)
    ;; std_msgs/Float32MultiArray _light
    (send _light :serialization-length)
    ;; nav_msgs/Odometry _odom
    (send _odom :serialization-length)
    ;; sensor_msgs/Range _sonar
    (send _sonar :serialization-length)
    ;; std_msgs/UInt16MultiArray _stream
    (send _stream :serialization-length)
    ;; std_msgs/UInt16 _touch_body
    (send _touch_body :serialization-length)
    ;; std_msgs/UInt16 _touch_head
    (send _touch_head :serialization-length)
    ;; std_msgs/Float32MultiArray _wheel_speed_cmd
    (send _wheel_speed_cmd :serialization-length)
    ;; std_msgs/Float32MultiArray _wheel_speed_back_emf
    (send _wheel_speed_back_emf :serialization-length)
    ;; std_msgs/Float32MultiArray _wheel_speed_opto
    (send _wheel_speed_opto :serialization-length)
    ;; std_msgs/Float32MultiArray _wheel_effort_pwm
    (send _wheel_effort_pwm :serialization-length)
    ;; geometry_msgs/Pose2D _body_pose
    (send _body_pose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; sensor_msgs/BatteryState _battery
       (send _battery :serialize s)
     ;; std_msgs/Float32MultiArray _cliff
       (send _cliff :serialize s)
     ;; std_msgs/UInt16 _dip
       (send _dip :serialize s)
     ;; std_msgs/UInt32 _flags
       (send _flags :serialize s)
     ;; sensor_msgs/Imu _imu_head
       (send _imu_head :serialize s)
     ;; sensor_msgs/Imu _imu_body
       (send _imu_body :serialize s)
     ;; sensor_msgs/JointState _kinematic_joints
       (send _kinematic_joints :serialize s)
     ;; std_msgs/Float32MultiArray _light
       (send _light :serialize s)
     ;; nav_msgs/Odometry _odom
       (send _odom :serialize s)
     ;; sensor_msgs/Range _sonar
       (send _sonar :serialize s)
     ;; std_msgs/UInt16MultiArray _stream
       (send _stream :serialize s)
     ;; std_msgs/UInt16 _touch_body
       (send _touch_body :serialize s)
     ;; std_msgs/UInt16 _touch_head
       (send _touch_head :serialize s)
     ;; std_msgs/Float32MultiArray _wheel_speed_cmd
       (send _wheel_speed_cmd :serialize s)
     ;; std_msgs/Float32MultiArray _wheel_speed_back_emf
       (send _wheel_speed_back_emf :serialize s)
     ;; std_msgs/Float32MultiArray _wheel_speed_opto
       (send _wheel_speed_opto :serialize s)
     ;; std_msgs/Float32MultiArray _wheel_effort_pwm
       (send _wheel_effort_pwm :serialize s)
     ;; geometry_msgs/Pose2D _body_pose
       (send _body_pose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; sensor_msgs/BatteryState _battery
     (send _battery :deserialize buf ptr-) (incf ptr- (send _battery :serialization-length))
   ;; std_msgs/Float32MultiArray _cliff
     (send _cliff :deserialize buf ptr-) (incf ptr- (send _cliff :serialization-length))
   ;; std_msgs/UInt16 _dip
     (send _dip :deserialize buf ptr-) (incf ptr- (send _dip :serialization-length))
   ;; std_msgs/UInt32 _flags
     (send _flags :deserialize buf ptr-) (incf ptr- (send _flags :serialization-length))
   ;; sensor_msgs/Imu _imu_head
     (send _imu_head :deserialize buf ptr-) (incf ptr- (send _imu_head :serialization-length))
   ;; sensor_msgs/Imu _imu_body
     (send _imu_body :deserialize buf ptr-) (incf ptr- (send _imu_body :serialization-length))
   ;; sensor_msgs/JointState _kinematic_joints
     (send _kinematic_joints :deserialize buf ptr-) (incf ptr- (send _kinematic_joints :serialization-length))
   ;; std_msgs/Float32MultiArray _light
     (send _light :deserialize buf ptr-) (incf ptr- (send _light :serialization-length))
   ;; nav_msgs/Odometry _odom
     (send _odom :deserialize buf ptr-) (incf ptr- (send _odom :serialization-length))
   ;; sensor_msgs/Range _sonar
     (send _sonar :deserialize buf ptr-) (incf ptr- (send _sonar :serialization-length))
   ;; std_msgs/UInt16MultiArray _stream
     (send _stream :deserialize buf ptr-) (incf ptr- (send _stream :serialization-length))
   ;; std_msgs/UInt16 _touch_body
     (send _touch_body :deserialize buf ptr-) (incf ptr- (send _touch_body :serialization-length))
   ;; std_msgs/UInt16 _touch_head
     (send _touch_head :deserialize buf ptr-) (incf ptr- (send _touch_head :serialization-length))
   ;; std_msgs/Float32MultiArray _wheel_speed_cmd
     (send _wheel_speed_cmd :deserialize buf ptr-) (incf ptr- (send _wheel_speed_cmd :serialization-length))
   ;; std_msgs/Float32MultiArray _wheel_speed_back_emf
     (send _wheel_speed_back_emf :deserialize buf ptr-) (incf ptr- (send _wheel_speed_back_emf :serialization-length))
   ;; std_msgs/Float32MultiArray _wheel_speed_opto
     (send _wheel_speed_opto :deserialize buf ptr-) (incf ptr- (send _wheel_speed_opto :serialization-length))
   ;; std_msgs/Float32MultiArray _wheel_effort_pwm
     (send _wheel_effort_pwm :deserialize buf ptr-) (incf ptr- (send _wheel_effort_pwm :serialization-length))
   ;; geometry_msgs/Pose2D _body_pose
     (send _body_pose :deserialize buf ptr-) (incf ptr- (send _body_pose :serialization-length))
   ;;
   self)
  )

(setf (get miro2_msg::sensors_package :md5sum-) "429d8257e8e981414c3f64c0a1074b4d")
(setf (get miro2_msg::sensors_package :datatype-) "miro2_msg/sensors_package")
(setf (get miro2_msg::sensors_package :definition-)
      "#	@section COPYRIGHT
#	Copyright (C) 2019 Consequential Robotics Ltd
#	
#	@section AUTHOR
#	Consequential Robotics http://consequentialrobotics.com
#	
#	@section LICENSE
#	For a full copy of the license agreement, and a complete
#	definition of \"The Software\", see LICENSE in the MDK root
#	directory.
#	
#	Subject to the terms of this Agreement, Consequential
#	Robotics grants to you a limited, non-exclusive, non-
#	transferable license, without right to sub-license, to use
#	\"The Software\" in accordance with this Agreement and any
#	other written agreement with Consequential Robotics.
#	Consequential Robotics does not transfer the title of \"The
#	Software\" to you; the license granted to you is not a sale.
#	This agreement is a binding legal agreement between
#	Consequential Robotics and the purchasers or users of \"The
#	Software\".
#	
#	THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY
#	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
#	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
#	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
#	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
#	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
#	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
#	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#	
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
# dim[0].label  = \"height\"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = \"width\"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = \"channel\"
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
# A covariance matrix of all zeros will be interpreted as \"covariance unknown\", and to use the
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

")



(provide :miro2_msg/sensors_package "429d8257e8e981414c3f64c0a1074b4d")


