; Auto-generated. Do not edit!


(cl:in-package miro2_msg-msg)


;//! \htmlinclude affect.msg.html

(cl:defclass <affect> (roslisp-msg-protocol:ros-message)
  ((valence
    :reader valence
    :initarg :valence
    :type cl:float
    :initform 0.0)
   (arousal
    :reader arousal
    :initarg :arousal
    :type cl:float
    :initform 0.0))
)

(cl:defclass affect (<affect>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <affect>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'affect)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name miro2_msg-msg:<affect> is deprecated: use miro2_msg-msg:affect instead.")))

(cl:ensure-generic-function 'valence-val :lambda-list '(m))
(cl:defmethod valence-val ((m <affect>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader miro2_msg-msg:valence-val is deprecated.  Use miro2_msg-msg:valence instead.")
  (valence m))

(cl:ensure-generic-function 'arousal-val :lambda-list '(m))
(cl:defmethod arousal-val ((m <affect>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader miro2_msg-msg:arousal-val is deprecated.  Use miro2_msg-msg:arousal instead.")
  (arousal m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <affect>) ostream)
  "Serializes a message object of type '<affect>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'valence))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'arousal))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <affect>) istream)
  "Deserializes a message object of type '<affect>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'valence) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'arousal) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<affect>)))
  "Returns string type for a message object of type '<affect>"
  "miro2_msg/affect")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'affect)))
  "Returns string type for a message object of type 'affect"
  "miro2_msg/affect")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<affect>)))
  "Returns md5sum for a message object of type '<affect>"
  "b9db7d9709bc98cc560b83ce0bc4f004")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'affect)))
  "Returns md5sum for a message object of type 'affect"
  "b9db7d9709bc98cc560b83ce0bc4f004")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<affect>)))
  "Returns full string definition for message of type '<affect>"
  (cl:format cl:nil "#	@section COPYRIGHT~%#	Copyright (C) 2019 Consequential Robotics Ltd~%#	~%#	@section AUTHOR~%#	Consequential Robotics http://consequentialrobotics.com~%#	~%#	@section LICENSE~%#	For a full copy of the license agreement, and a complete~%#	definition of \"The Software\", see LICENSE in the MDK root~%#	directory.~%#	~%#	Subject to the terms of this Agreement, Consequential~%#	Robotics grants to you a limited, non-exclusive, non-~%#	transferable license, without right to sub-license, to use~%#	\"The Software\" in accordance with this Agreement and any~%#	other written agreement with Consequential Robotics.~%#	Consequential Robotics does not transfer the title of \"The~%#	Software\" to you; the license granted to you is not a sale.~%#	This agreement is a binding legal agreement between~%#	Consequential Robotics and the purchasers or users of \"The~%#	Software\".~%#	~%#	THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY~%#	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE~%#	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR~%#	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS~%#	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR~%#	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR~%#	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE~%#	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.~%#	~%#~%#	The \"affect\" state is two-dimensional, encoding valence~%#	(-ve = sad, +ve = happy) and arousal (-ve = relaxed, +ve = alert).~%#	The states are usually driven by signals entering the robot's~%#	sensory systems, but can also be driven directly by other systems.~%~%float32 valence~%float32 arousal~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'affect)))
  "Returns full string definition for message of type 'affect"
  (cl:format cl:nil "#	@section COPYRIGHT~%#	Copyright (C) 2019 Consequential Robotics Ltd~%#	~%#	@section AUTHOR~%#	Consequential Robotics http://consequentialrobotics.com~%#	~%#	@section LICENSE~%#	For a full copy of the license agreement, and a complete~%#	definition of \"The Software\", see LICENSE in the MDK root~%#	directory.~%#	~%#	Subject to the terms of this Agreement, Consequential~%#	Robotics grants to you a limited, non-exclusive, non-~%#	transferable license, without right to sub-license, to use~%#	\"The Software\" in accordance with this Agreement and any~%#	other written agreement with Consequential Robotics.~%#	Consequential Robotics does not transfer the title of \"The~%#	Software\" to you; the license granted to you is not a sale.~%#	This agreement is a binding legal agreement between~%#	Consequential Robotics and the purchasers or users of \"The~%#	Software\".~%#	~%#	THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY~%#	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE~%#	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR~%#	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS~%#	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR~%#	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR~%#	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE~%#	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.~%#	~%#~%#	The \"affect\" state is two-dimensional, encoding valence~%#	(-ve = sad, +ve = happy) and arousal (-ve = relaxed, +ve = alert).~%#	The states are usually driven by signals entering the robot's~%#	sensory systems, but can also be driven directly by other systems.~%~%float32 valence~%float32 arousal~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <affect>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <affect>))
  "Converts a ROS message object to a list"
  (cl:list 'affect
    (cl:cons ':valence (valence msg))
    (cl:cons ':arousal (arousal msg))
))
