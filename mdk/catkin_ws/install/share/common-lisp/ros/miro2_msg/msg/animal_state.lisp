; Auto-generated. Do not edit!


(cl:in-package miro2_msg-msg)


;//! \htmlinclude animal_state.msg.html

(cl:defclass <animal_state> (roslisp-msg-protocol:ros-message)
  ((flags
    :reader flags
    :initarg :flags
    :type cl:integer
    :initform 0)
   (emotion
    :reader emotion
    :initarg :emotion
    :type miro2_msg-msg:affect
    :initform (cl:make-instance 'miro2_msg-msg:affect))
   (mood
    :reader mood
    :initarg :mood
    :type miro2_msg-msg:affect
    :initform (cl:make-instance 'miro2_msg-msg:affect))
   (sleep
    :reader sleep
    :initarg :sleep
    :type miro2_msg-msg:sleep
    :initform (cl:make-instance 'miro2_msg-msg:sleep))
   (time_of_day
    :reader time_of_day
    :initarg :time_of_day
    :type cl:float
    :initform 0.0)
   (sound_level
    :reader sound_level
    :initarg :sound_level
    :type cl:float
    :initform 0.0))
)

(cl:defclass animal_state (<animal_state>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <animal_state>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'animal_state)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name miro2_msg-msg:<animal_state> is deprecated: use miro2_msg-msg:animal_state instead.")))

(cl:ensure-generic-function 'flags-val :lambda-list '(m))
(cl:defmethod flags-val ((m <animal_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader miro2_msg-msg:flags-val is deprecated.  Use miro2_msg-msg:flags instead.")
  (flags m))

(cl:ensure-generic-function 'emotion-val :lambda-list '(m))
(cl:defmethod emotion-val ((m <animal_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader miro2_msg-msg:emotion-val is deprecated.  Use miro2_msg-msg:emotion instead.")
  (emotion m))

(cl:ensure-generic-function 'mood-val :lambda-list '(m))
(cl:defmethod mood-val ((m <animal_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader miro2_msg-msg:mood-val is deprecated.  Use miro2_msg-msg:mood instead.")
  (mood m))

(cl:ensure-generic-function 'sleep-val :lambda-list '(m))
(cl:defmethod sleep-val ((m <animal_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader miro2_msg-msg:sleep-val is deprecated.  Use miro2_msg-msg:sleep instead.")
  (sleep m))

(cl:ensure-generic-function 'time_of_day-val :lambda-list '(m))
(cl:defmethod time_of_day-val ((m <animal_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader miro2_msg-msg:time_of_day-val is deprecated.  Use miro2_msg-msg:time_of_day instead.")
  (time_of_day m))

(cl:ensure-generic-function 'sound_level-val :lambda-list '(m))
(cl:defmethod sound_level-val ((m <animal_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader miro2_msg-msg:sound_level-val is deprecated.  Use miro2_msg-msg:sound_level instead.")
  (sound_level m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <animal_state>) ostream)
  "Serializes a message object of type '<animal_state>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'flags)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'emotion) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'mood) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'sleep) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'time_of_day))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'sound_level))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <animal_state>) istream)
  "Deserializes a message object of type '<animal_state>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'flags)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'flags)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'flags)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'flags)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'emotion) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'mood) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'sleep) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'time_of_day) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'sound_level) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<animal_state>)))
  "Returns string type for a message object of type '<animal_state>"
  "miro2_msg/animal_state")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'animal_state)))
  "Returns string type for a message object of type 'animal_state"
  "miro2_msg/animal_state")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<animal_state>)))
  "Returns md5sum for a message object of type '<animal_state>"
  "02b89a84b06f59e91819662e7c3d6b0e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'animal_state)))
  "Returns md5sum for a message object of type 'animal_state"
  "02b89a84b06f59e91819662e7c3d6b0e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<animal_state>)))
  "Returns full string definition for message of type '<animal_state>"
  (cl:format cl:nil "#	@section COPYRIGHT~%#	Copyright (C) 2019 Consequential Robotics Ltd~%#	~%#	@section AUTHOR~%#	Consequential Robotics http://consequentialrobotics.com~%#	~%#	@section LICENSE~%#	For a full copy of the license agreement, and a complete~%#	definition of \"The Software\", see LICENSE in the MDK root~%#	directory.~%#	~%#	Subject to the terms of this Agreement, Consequential~%#	Robotics grants to you a limited, non-exclusive, non-~%#	transferable license, without right to sub-license, to use~%#	\"The Software\" in accordance with this Agreement and any~%#	other written agreement with Consequential Robotics.~%#	Consequential Robotics does not transfer the title of \"The~%#	Software\" to you; the license granted to you is not a sale.~%#	This agreement is a binding legal agreement between~%#	Consequential Robotics and the purchasers or users of \"The~%#	Software\".~%#	~%#	THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY~%#	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE~%#	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR~%#	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS~%#	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR~%#	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR~%#	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE~%#	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.~%#	~%#~%# the \"animal_state\" is the state of the animal aspects of the model~%# which includes \"affect\" values for emotion (quickly changing) and~%# mood (slowly changing) and a \"sleep\" value (also slowly changing),~%# and the animal's estimate of time. this state may also include, in~%# future, physical states such as temperature.~%~%#	DOCLINK ANIMAL STATE FLAGS~%#~%#	Some flags are included here because parts of the implementation~%#	are in separate nodes that read this topic in order to determine~%#	how they should behave, and their behaviour is affected by flags.~%#~%#	The values of these flags are defined in miro2.constants.~%uint32 flags~%~%# affective states~%affect emotion~%affect mood~%~%# sleep state~%sleep sleep~%~%# normalised time of day (0.0 -> 1.0)~%float32 time_of_day~%~%# normalised ambient sound level (0.0 -> 1.0)~%# < 0.01 : pretty quiet~%# 0.01 : normal ambient music~%# 0.02-0.03 : loud music~%# 0.05 : very loud music~%# > 0.1 : System of a Down~%float32 sound_level~%~%~%================================================================================~%MSG: miro2_msg/affect~%#	@section COPYRIGHT~%#	Copyright (C) 2019 Consequential Robotics Ltd~%#	~%#	@section AUTHOR~%#	Consequential Robotics http://consequentialrobotics.com~%#	~%#	@section LICENSE~%#	For a full copy of the license agreement, and a complete~%#	definition of \"The Software\", see LICENSE in the MDK root~%#	directory.~%#	~%#	Subject to the terms of this Agreement, Consequential~%#	Robotics grants to you a limited, non-exclusive, non-~%#	transferable license, without right to sub-license, to use~%#	\"The Software\" in accordance with this Agreement and any~%#	other written agreement with Consequential Robotics.~%#	Consequential Robotics does not transfer the title of \"The~%#	Software\" to you; the license granted to you is not a sale.~%#	This agreement is a binding legal agreement between~%#	Consequential Robotics and the purchasers or users of \"The~%#	Software\".~%#	~%#	THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY~%#	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE~%#	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR~%#	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS~%#	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR~%#	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR~%#	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE~%#	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.~%#	~%#~%#	The \"affect\" state is two-dimensional, encoding valence~%#	(-ve = sad, +ve = happy) and arousal (-ve = relaxed, +ve = alert).~%#	The states are usually driven by signals entering the robot's~%#	sensory systems, but can also be driven directly by other systems.~%~%float32 valence~%float32 arousal~%~%~%================================================================================~%MSG: miro2_msg/sleep~%#	@section COPYRIGHT~%#	Copyright (C) 2019 Consequential Robotics Ltd~%#	~%#	@section AUTHOR~%#	Consequential Robotics http://consequentialrobotics.com~%#	~%#	@section LICENSE~%#	For a full copy of the license agreement, and a complete~%#	definition of \"The Software\", see LICENSE in the MDK root~%#	directory.~%#	~%#	Subject to the terms of this Agreement, Consequential~%#	Robotics grants to you a limited, non-exclusive, non-~%#	transferable license, without right to sub-license, to use~%#	\"The Software\" in accordance with this Agreement and any~%#	other written agreement with Consequential Robotics.~%#	Consequential Robotics does not transfer the title of \"The~%#	Software\" to you; the license granted to you is not a sale.~%#	This agreement is a binding legal agreement between~%#	Consequential Robotics and the purchasers or users of \"The~%#	Software\".~%#	~%#	THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY~%#	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE~%#	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR~%#	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS~%#	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR~%#	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR~%#	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE~%#	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.~%#	~%#~%#	The \"sleep\" state is two-dimensional, encoding \"wakefulness\"~%#	(0.0 to 1.0, what it sounds like) and \"pressure\" (0.0 to 1.0,~%#	tendency to move towards reduced wakefulness). The two states~%#	evolve together to implement a relaxation oscillator.~%~%float32 wakefulness~%float32 pressure~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'animal_state)))
  "Returns full string definition for message of type 'animal_state"
  (cl:format cl:nil "#	@section COPYRIGHT~%#	Copyright (C) 2019 Consequential Robotics Ltd~%#	~%#	@section AUTHOR~%#	Consequential Robotics http://consequentialrobotics.com~%#	~%#	@section LICENSE~%#	For a full copy of the license agreement, and a complete~%#	definition of \"The Software\", see LICENSE in the MDK root~%#	directory.~%#	~%#	Subject to the terms of this Agreement, Consequential~%#	Robotics grants to you a limited, non-exclusive, non-~%#	transferable license, without right to sub-license, to use~%#	\"The Software\" in accordance with this Agreement and any~%#	other written agreement with Consequential Robotics.~%#	Consequential Robotics does not transfer the title of \"The~%#	Software\" to you; the license granted to you is not a sale.~%#	This agreement is a binding legal agreement between~%#	Consequential Robotics and the purchasers or users of \"The~%#	Software\".~%#	~%#	THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY~%#	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE~%#	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR~%#	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS~%#	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR~%#	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR~%#	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE~%#	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.~%#	~%#~%# the \"animal_state\" is the state of the animal aspects of the model~%# which includes \"affect\" values for emotion (quickly changing) and~%# mood (slowly changing) and a \"sleep\" value (also slowly changing),~%# and the animal's estimate of time. this state may also include, in~%# future, physical states such as temperature.~%~%#	DOCLINK ANIMAL STATE FLAGS~%#~%#	Some flags are included here because parts of the implementation~%#	are in separate nodes that read this topic in order to determine~%#	how they should behave, and their behaviour is affected by flags.~%#~%#	The values of these flags are defined in miro2.constants.~%uint32 flags~%~%# affective states~%affect emotion~%affect mood~%~%# sleep state~%sleep sleep~%~%# normalised time of day (0.0 -> 1.0)~%float32 time_of_day~%~%# normalised ambient sound level (0.0 -> 1.0)~%# < 0.01 : pretty quiet~%# 0.01 : normal ambient music~%# 0.02-0.03 : loud music~%# 0.05 : very loud music~%# > 0.1 : System of a Down~%float32 sound_level~%~%~%================================================================================~%MSG: miro2_msg/affect~%#	@section COPYRIGHT~%#	Copyright (C) 2019 Consequential Robotics Ltd~%#	~%#	@section AUTHOR~%#	Consequential Robotics http://consequentialrobotics.com~%#	~%#	@section LICENSE~%#	For a full copy of the license agreement, and a complete~%#	definition of \"The Software\", see LICENSE in the MDK root~%#	directory.~%#	~%#	Subject to the terms of this Agreement, Consequential~%#	Robotics grants to you a limited, non-exclusive, non-~%#	transferable license, without right to sub-license, to use~%#	\"The Software\" in accordance with this Agreement and any~%#	other written agreement with Consequential Robotics.~%#	Consequential Robotics does not transfer the title of \"The~%#	Software\" to you; the license granted to you is not a sale.~%#	This agreement is a binding legal agreement between~%#	Consequential Robotics and the purchasers or users of \"The~%#	Software\".~%#	~%#	THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY~%#	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE~%#	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR~%#	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS~%#	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR~%#	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR~%#	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE~%#	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.~%#	~%#~%#	The \"affect\" state is two-dimensional, encoding valence~%#	(-ve = sad, +ve = happy) and arousal (-ve = relaxed, +ve = alert).~%#	The states are usually driven by signals entering the robot's~%#	sensory systems, but can also be driven directly by other systems.~%~%float32 valence~%float32 arousal~%~%~%================================================================================~%MSG: miro2_msg/sleep~%#	@section COPYRIGHT~%#	Copyright (C) 2019 Consequential Robotics Ltd~%#	~%#	@section AUTHOR~%#	Consequential Robotics http://consequentialrobotics.com~%#	~%#	@section LICENSE~%#	For a full copy of the license agreement, and a complete~%#	definition of \"The Software\", see LICENSE in the MDK root~%#	directory.~%#	~%#	Subject to the terms of this Agreement, Consequential~%#	Robotics grants to you a limited, non-exclusive, non-~%#	transferable license, without right to sub-license, to use~%#	\"The Software\" in accordance with this Agreement and any~%#	other written agreement with Consequential Robotics.~%#	Consequential Robotics does not transfer the title of \"The~%#	Software\" to you; the license granted to you is not a sale.~%#	This agreement is a binding legal agreement between~%#	Consequential Robotics and the purchasers or users of \"The~%#	Software\".~%#	~%#	THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY~%#	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE~%#	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR~%#	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS~%#	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR~%#	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR~%#	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE~%#	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.~%#	~%#~%#	The \"sleep\" state is two-dimensional, encoding \"wakefulness\"~%#	(0.0 to 1.0, what it sounds like) and \"pressure\" (0.0 to 1.0,~%#	tendency to move towards reduced wakefulness). The two states~%#	evolve together to implement a relaxation oscillator.~%~%float32 wakefulness~%float32 pressure~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <animal_state>))
  (cl:+ 0
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'emotion))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'mood))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'sleep))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <animal_state>))
  "Converts a ROS message object to a list"
  (cl:list 'animal_state
    (cl:cons ':flags (flags msg))
    (cl:cons ':emotion (emotion msg))
    (cl:cons ':mood (mood msg))
    (cl:cons ':sleep (sleep msg))
    (cl:cons ':time_of_day (time_of_day msg))
    (cl:cons ':sound_level (sound_level msg))
))
