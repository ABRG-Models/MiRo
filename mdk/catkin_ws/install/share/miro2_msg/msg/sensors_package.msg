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




