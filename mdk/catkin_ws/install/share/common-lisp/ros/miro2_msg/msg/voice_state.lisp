; Auto-generated. Do not edit!


(cl:in-package miro2_msg-msg)


;//! \htmlinclude voice_state.msg.html

(cl:defclass <voice_state> (roslisp-msg-protocol:ros-message)
  ((breathing_phase
    :reader breathing_phase
    :initarg :breathing_phase
    :type cl:float
    :initform 0.0)
   (vocalising
    :reader vocalising
    :initarg :vocalising
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass voice_state (<voice_state>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <voice_state>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'voice_state)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name miro2_msg-msg:<voice_state> is deprecated: use miro2_msg-msg:voice_state instead.")))

(cl:ensure-generic-function 'breathing_phase-val :lambda-list '(m))
(cl:defmethod breathing_phase-val ((m <voice_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader miro2_msg-msg:breathing_phase-val is deprecated.  Use miro2_msg-msg:breathing_phase instead.")
  (breathing_phase m))

(cl:ensure-generic-function 'vocalising-val :lambda-list '(m))
(cl:defmethod vocalising-val ((m <voice_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader miro2_msg-msg:vocalising-val is deprecated.  Use miro2_msg-msg:vocalising instead.")
  (vocalising m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <voice_state>) ostream)
  "Serializes a message object of type '<voice_state>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'breathing_phase))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'vocalising) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <voice_state>) istream)
  "Deserializes a message object of type '<voice_state>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'breathing_phase) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'vocalising) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<voice_state>)))
  "Returns string type for a message object of type '<voice_state>"
  "miro2_msg/voice_state")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'voice_state)))
  "Returns string type for a message object of type 'voice_state"
  "miro2_msg/voice_state")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<voice_state>)))
  "Returns md5sum for a message object of type '<voice_state>"
  "e821a66f37dcfd027ec1d69a1734ae31")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'voice_state)))
  "Returns md5sum for a message object of type 'voice_state"
  "e821a66f37dcfd027ec1d69a1734ae31")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<voice_state>)))
  "Returns full string definition for message of type '<voice_state>"
  (cl:format cl:nil "#	@section COPYRIGHT~%#	Copyright (C) 2019 Consequential Robotics Ltd~%#	~%#	@section AUTHOR~%#	Consequential Robotics http://consequentialrobotics.com~%#	~%#	@section LICENSE~%#	For a full copy of the license agreement, see LICENSE in the~%#	MDK root directory.~%#	~%#	Subject to the terms of this Agreement, Consequential~%#	Robotics grants to you a limited, non-exclusive, non-~%#	transferable license, without right to sub-license, to use~%#	MIRO Developer Kit in accordance with this Agreement and any~%#	other written agreement with Consequential Robotics.~%#	Consequential Robotics does not transfer the title of MIRO~%#	Developer Kit to you; the license granted to you is not a~%#	sale. This agreement is a binding legal agreement between~%#	Consequential Robotics and the purchasers or users of MIRO~%#	Developer Kit.~%#	~%#	THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY~%#	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE~%#	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR~%#	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS~%#	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR~%#	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR~%#	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE~%#	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.~%~%float32 breathing_phase~%bool vocalising~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'voice_state)))
  "Returns full string definition for message of type 'voice_state"
  (cl:format cl:nil "#	@section COPYRIGHT~%#	Copyright (C) 2019 Consequential Robotics Ltd~%#	~%#	@section AUTHOR~%#	Consequential Robotics http://consequentialrobotics.com~%#	~%#	@section LICENSE~%#	For a full copy of the license agreement, see LICENSE in the~%#	MDK root directory.~%#	~%#	Subject to the terms of this Agreement, Consequential~%#	Robotics grants to you a limited, non-exclusive, non-~%#	transferable license, without right to sub-license, to use~%#	MIRO Developer Kit in accordance with this Agreement and any~%#	other written agreement with Consequential Robotics.~%#	Consequential Robotics does not transfer the title of MIRO~%#	Developer Kit to you; the license granted to you is not a~%#	sale. This agreement is a binding legal agreement between~%#	Consequential Robotics and the purchasers or users of MIRO~%#	Developer Kit.~%#	~%#	THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY~%#	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE~%#	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR~%#	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS~%#	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR~%#	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR~%#	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE~%#	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.~%~%float32 breathing_phase~%bool vocalising~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <voice_state>))
  (cl:+ 0
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <voice_state>))
  "Converts a ROS message object to a list"
  (cl:list 'voice_state
    (cl:cons ':breathing_phase (breathing_phase msg))
    (cl:cons ':vocalising (vocalising msg))
))
