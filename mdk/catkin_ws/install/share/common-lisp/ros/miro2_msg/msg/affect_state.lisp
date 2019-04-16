; Auto-generated. Do not edit!


(cl:in-package miro2_msg-msg)


;//! \htmlinclude affect_state.msg.html

(cl:defclass <affect_state> (roslisp-msg-protocol:ros-message)
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
    :initform (cl:make-instance 'miro2_msg-msg:sleep)))
)

(cl:defclass affect_state (<affect_state>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <affect_state>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'affect_state)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name miro2_msg-msg:<affect_state> is deprecated: use miro2_msg-msg:affect_state instead.")))

(cl:ensure-generic-function 'flags-val :lambda-list '(m))
(cl:defmethod flags-val ((m <affect_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader miro2_msg-msg:flags-val is deprecated.  Use miro2_msg-msg:flags instead.")
  (flags m))

(cl:ensure-generic-function 'emotion-val :lambda-list '(m))
(cl:defmethod emotion-val ((m <affect_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader miro2_msg-msg:emotion-val is deprecated.  Use miro2_msg-msg:emotion instead.")
  (emotion m))

(cl:ensure-generic-function 'mood-val :lambda-list '(m))
(cl:defmethod mood-val ((m <affect_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader miro2_msg-msg:mood-val is deprecated.  Use miro2_msg-msg:mood instead.")
  (mood m))

(cl:ensure-generic-function 'sleep-val :lambda-list '(m))
(cl:defmethod sleep-val ((m <affect_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader miro2_msg-msg:sleep-val is deprecated.  Use miro2_msg-msg:sleep instead.")
  (sleep m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <affect_state>) ostream)
  "Serializes a message object of type '<affect_state>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'flags)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'emotion) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'mood) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'sleep) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <affect_state>) istream)
  "Deserializes a message object of type '<affect_state>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'flags)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'flags)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'flags)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'flags)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'emotion) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'mood) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'sleep) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<affect_state>)))
  "Returns string type for a message object of type '<affect_state>"
  "miro2_msg/affect_state")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'affect_state)))
  "Returns string type for a message object of type 'affect_state"
  "miro2_msg/affect_state")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<affect_state>)))
  "Returns md5sum for a message object of type '<affect_state>"
  "8eba44c465d72613e49b25d7a944efcf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'affect_state)))
  "Returns md5sum for a message object of type 'affect_state"
  "8eba44c465d72613e49b25d7a944efcf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<affect_state>)))
  "Returns full string definition for message of type '<affect_state>"
  (cl:format cl:nil "#	@section COPYRIGHT~%#	Copyright (C) 2019 Consequential Robotics Ltd~%#	~%#	@section AUTHOR~%#	Consequential Robotics http://consequentialrobotics.com~%#	~%#	@section LICENSE~%#	For a full copy of the license agreement, and a complete~%#	definition of \"The Software\", see LICENSE in the MDK root~%#	directory.~%#	~%#	Subject to the terms of this Agreement, Consequential~%#	Robotics grants to you a limited, non-exclusive, non-~%#	transferable license, without right to sub-license, to use~%#	\"The Software\" in accordance with this Agreement and any~%#	other written agreement with Consequential Robotics.~%#	Consequential Robotics does not transfer the title of \"The~%#	Software\" to you; the license granted to you is not a sale.~%#	This agreement is a binding legal agreement between~%#	Consequential Robotics and the purchasers or users of \"The~%#	Software\".~%#	~%#	THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY~%#	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE~%#	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR~%#	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS~%#	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR~%#	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR~%#	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE~%#	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.~%#	~%~%#	DOCLINK AFFECT FLAGS~%#~%#	The values of these flags are defined in miro2.constants~%#~%#	AFFECT_EXPRESS_THROUGH_VOICE~%#		Enable vocal expression of affective state~%#~%#	AFFECT_EXPRESS_THROUGH_BODY~%#		Enable body expression of affective state~%uint32 flags~%~%# the \"affect_state\" is the complete affective state, including~%# \"affect\" values for emotion (quickly changing) and mood (slowly~%# changing) and a \"sleep\" value (also slowly changing).~%affect emotion~%affect mood~%sleep sleep~%~%~%================================================================================~%MSG: miro2_msg/affect~%#	@section COPYRIGHT~%#	Copyright (C) 2019 Consequential Robotics Ltd~%#	~%#	@section AUTHOR~%#	Consequential Robotics http://consequentialrobotics.com~%#	~%#	@section LICENSE~%#	For a full copy of the license agreement, and a complete~%#	definition of \"The Software\", see LICENSE in the MDK root~%#	directory.~%#	~%#	Subject to the terms of this Agreement, Consequential~%#	Robotics grants to you a limited, non-exclusive, non-~%#	transferable license, without right to sub-license, to use~%#	\"The Software\" in accordance with this Agreement and any~%#	other written agreement with Consequential Robotics.~%#	Consequential Robotics does not transfer the title of \"The~%#	Software\" to you; the license granted to you is not a sale.~%#	This agreement is a binding legal agreement between~%#	Consequential Robotics and the purchasers or users of \"The~%#	Software\".~%#	~%#	THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY~%#	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE~%#	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR~%#	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS~%#	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR~%#	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR~%#	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE~%#	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.~%#	~%#~%#	The \"affect\" state is two-dimensional, encoding valence~%#	(-ve = sad, +ve = happy) and arousal (-ve = relaxed, +ve = alert).~%#	The states are usually driven by signals entering the robot's~%#	sensory systems, but can also be driven directly by other systems.~%~%float32 valence~%float32 arousal~%~%~%================================================================================~%MSG: miro2_msg/sleep~%#	@section COPYRIGHT~%#	Copyright (C) 2019 Consequential Robotics Ltd~%#	~%#	@section AUTHOR~%#	Consequential Robotics http://consequentialrobotics.com~%#	~%#	@section LICENSE~%#	For a full copy of the license agreement, and a complete~%#	definition of \"The Software\", see LICENSE in the MDK root~%#	directory.~%#	~%#	Subject to the terms of this Agreement, Consequential~%#	Robotics grants to you a limited, non-exclusive, non-~%#	transferable license, without right to sub-license, to use~%#	\"The Software\" in accordance with this Agreement and any~%#	other written agreement with Consequential Robotics.~%#	Consequential Robotics does not transfer the title of \"The~%#	Software\" to you; the license granted to you is not a sale.~%#	This agreement is a binding legal agreement between~%#	Consequential Robotics and the purchasers or users of \"The~%#	Software\".~%#	~%#	THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY~%#	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE~%#	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR~%#	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS~%#	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR~%#	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR~%#	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE~%#	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.~%#	~%#~%#	The \"sleep\" state is two-dimensional, encoding \"wakefulness\"~%#	(0.0 to 1.0, what it sounds like) and \"pressure\" (0.0 to 1.0,~%#	tendency to move towards reduced wakefulness). The two states~%#	evolve together to implement a relaxation oscillator.~%~%float32 wakefulness~%float32 pressure~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'affect_state)))
  "Returns full string definition for message of type 'affect_state"
  (cl:format cl:nil "#	@section COPYRIGHT~%#	Copyright (C) 2019 Consequential Robotics Ltd~%#	~%#	@section AUTHOR~%#	Consequential Robotics http://consequentialrobotics.com~%#	~%#	@section LICENSE~%#	For a full copy of the license agreement, and a complete~%#	definition of \"The Software\", see LICENSE in the MDK root~%#	directory.~%#	~%#	Subject to the terms of this Agreement, Consequential~%#	Robotics grants to you a limited, non-exclusive, non-~%#	transferable license, without right to sub-license, to use~%#	\"The Software\" in accordance with this Agreement and any~%#	other written agreement with Consequential Robotics.~%#	Consequential Robotics does not transfer the title of \"The~%#	Software\" to you; the license granted to you is not a sale.~%#	This agreement is a binding legal agreement between~%#	Consequential Robotics and the purchasers or users of \"The~%#	Software\".~%#	~%#	THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY~%#	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE~%#	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR~%#	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS~%#	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR~%#	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR~%#	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE~%#	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.~%#	~%~%#	DOCLINK AFFECT FLAGS~%#~%#	The values of these flags are defined in miro2.constants~%#~%#	AFFECT_EXPRESS_THROUGH_VOICE~%#		Enable vocal expression of affective state~%#~%#	AFFECT_EXPRESS_THROUGH_BODY~%#		Enable body expression of affective state~%uint32 flags~%~%# the \"affect_state\" is the complete affective state, including~%# \"affect\" values for emotion (quickly changing) and mood (slowly~%# changing) and a \"sleep\" value (also slowly changing).~%affect emotion~%affect mood~%sleep sleep~%~%~%================================================================================~%MSG: miro2_msg/affect~%#	@section COPYRIGHT~%#	Copyright (C) 2019 Consequential Robotics Ltd~%#	~%#	@section AUTHOR~%#	Consequential Robotics http://consequentialrobotics.com~%#	~%#	@section LICENSE~%#	For a full copy of the license agreement, and a complete~%#	definition of \"The Software\", see LICENSE in the MDK root~%#	directory.~%#	~%#	Subject to the terms of this Agreement, Consequential~%#	Robotics grants to you a limited, non-exclusive, non-~%#	transferable license, without right to sub-license, to use~%#	\"The Software\" in accordance with this Agreement and any~%#	other written agreement with Consequential Robotics.~%#	Consequential Robotics does not transfer the title of \"The~%#	Software\" to you; the license granted to you is not a sale.~%#	This agreement is a binding legal agreement between~%#	Consequential Robotics and the purchasers or users of \"The~%#	Software\".~%#	~%#	THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY~%#	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE~%#	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR~%#	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS~%#	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR~%#	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR~%#	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE~%#	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.~%#	~%#~%#	The \"affect\" state is two-dimensional, encoding valence~%#	(-ve = sad, +ve = happy) and arousal (-ve = relaxed, +ve = alert).~%#	The states are usually driven by signals entering the robot's~%#	sensory systems, but can also be driven directly by other systems.~%~%float32 valence~%float32 arousal~%~%~%================================================================================~%MSG: miro2_msg/sleep~%#	@section COPYRIGHT~%#	Copyright (C) 2019 Consequential Robotics Ltd~%#	~%#	@section AUTHOR~%#	Consequential Robotics http://consequentialrobotics.com~%#	~%#	@section LICENSE~%#	For a full copy of the license agreement, and a complete~%#	definition of \"The Software\", see LICENSE in the MDK root~%#	directory.~%#	~%#	Subject to the terms of this Agreement, Consequential~%#	Robotics grants to you a limited, non-exclusive, non-~%#	transferable license, without right to sub-license, to use~%#	\"The Software\" in accordance with this Agreement and any~%#	other written agreement with Consequential Robotics.~%#	Consequential Robotics does not transfer the title of \"The~%#	Software\" to you; the license granted to you is not a sale.~%#	This agreement is a binding legal agreement between~%#	Consequential Robotics and the purchasers or users of \"The~%#	Software\".~%#	~%#	THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY~%#	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE~%#	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR~%#	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS~%#	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR~%#	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR~%#	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE~%#	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.~%#	~%#~%#	The \"sleep\" state is two-dimensional, encoding \"wakefulness\"~%#	(0.0 to 1.0, what it sounds like) and \"pressure\" (0.0 to 1.0,~%#	tendency to move towards reduced wakefulness). The two states~%#	evolve together to implement a relaxation oscillator.~%~%float32 wakefulness~%float32 pressure~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <affect_state>))
  (cl:+ 0
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'emotion))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'mood))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'sleep))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <affect_state>))
  "Converts a ROS message object to a list"
  (cl:list 'affect_state
    (cl:cons ':flags (flags msg))
    (cl:cons ':emotion (emotion msg))
    (cl:cons ':mood (mood msg))
    (cl:cons ':sleep (sleep msg))
))
