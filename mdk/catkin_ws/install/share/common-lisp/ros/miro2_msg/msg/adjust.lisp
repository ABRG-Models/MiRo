; Auto-generated. Do not edit!


(cl:in-package miro2_msg-msg)


;//! \htmlinclude adjust.msg.html

(cl:defclass <adjust> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type cl:float
    :initform 0.0)
   (gamma
    :reader gamma
    :initarg :gamma
    :type cl:float
    :initform 0.0))
)

(cl:defclass adjust (<adjust>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <adjust>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'adjust)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name miro2_msg-msg:<adjust> is deprecated: use miro2_msg-msg:adjust instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <adjust>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader miro2_msg-msg:data-val is deprecated.  Use miro2_msg-msg:data instead.")
  (data m))

(cl:ensure-generic-function 'gamma-val :lambda-list '(m))
(cl:defmethod gamma-val ((m <adjust>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader miro2_msg-msg:gamma-val is deprecated.  Use miro2_msg-msg:gamma instead.")
  (gamma m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <adjust>) ostream)
  "Serializes a message object of type '<adjust>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'gamma))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <adjust>) istream)
  "Deserializes a message object of type '<adjust>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'data) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'gamma) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<adjust>)))
  "Returns string type for a message object of type '<adjust>"
  "miro2_msg/adjust")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'adjust)))
  "Returns string type for a message object of type 'adjust"
  "miro2_msg/adjust")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<adjust>)))
  "Returns md5sum for a message object of type '<adjust>"
  "f57be85f314b6a6c4ab78af0ca827a06")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'adjust)))
  "Returns md5sum for a message object of type 'adjust"
  "f57be85f314b6a6c4ab78af0ca827a06")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<adjust>)))
  "Returns full string definition for message of type '<adjust>"
  (cl:format cl:nil "#	@section COPYRIGHT~%#	Copyright (C) 2019 Consequential Robotics Ltd~%#	~%#	@section AUTHOR~%#	Consequential Robotics http://consequentialrobotics.com~%#	~%#	@section LICENSE~%#	For a full copy of the license agreement, and a complete~%#	definition of \"The Software\", see LICENSE in the MDK root~%#	directory.~%#	~%#	Subject to the terms of this Agreement, Consequential~%#	Robotics grants to you a limited, non-exclusive, non-~%#	transferable license, without right to sub-license, to use~%#	\"The Software\" in accordance with this Agreement and any~%#	other written agreement with Consequential Robotics.~%#	Consequential Robotics does not transfer the title of \"The~%#	Software\" to you; the license granted to you is not a sale.~%#	This agreement is a binding legal agreement between~%#	Consequential Robotics and the purchasers or users of \"The~%#	Software\".~%#	~%#	THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY~%#	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE~%#	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR~%#	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS~%#	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR~%#	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR~%#	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE~%#	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.~%#	~%#~%#	Adjust message provides a route for directly adjusting~%#	a state of the biomimetic model. There are two ways to~%#	specify an adjustment, selected independently for each~%#	adjustment channel.~%#~%#	1) Provide a target value in \"data\" and a \"gamma\" value~%#	between 0 and 1 to cause the state to approach the target:~%#~%#	(at 50Hz)~%#	state += gamma * (data - state)~%#~%#	2) Provide a delta value in \"data\" and set \"gamma\"~%#	to -1 to indicate this drive mode:~%#~%#	(at 50Hz)~%#	state += data~%#~%#	Understood values of gamma, therefore, are:~%#	   -1 : add \"data\" to state~%#	    0 : do nothing~%#	  0-1 : move state towards \"data\"~%#	    1 : instantly set state to \"data\"~%~%float32 data~%float32 gamma~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'adjust)))
  "Returns full string definition for message of type 'adjust"
  (cl:format cl:nil "#	@section COPYRIGHT~%#	Copyright (C) 2019 Consequential Robotics Ltd~%#	~%#	@section AUTHOR~%#	Consequential Robotics http://consequentialrobotics.com~%#	~%#	@section LICENSE~%#	For a full copy of the license agreement, and a complete~%#	definition of \"The Software\", see LICENSE in the MDK root~%#	directory.~%#	~%#	Subject to the terms of this Agreement, Consequential~%#	Robotics grants to you a limited, non-exclusive, non-~%#	transferable license, without right to sub-license, to use~%#	\"The Software\" in accordance with this Agreement and any~%#	other written agreement with Consequential Robotics.~%#	Consequential Robotics does not transfer the title of \"The~%#	Software\" to you; the license granted to you is not a sale.~%#	This agreement is a binding legal agreement between~%#	Consequential Robotics and the purchasers or users of \"The~%#	Software\".~%#	~%#	THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY~%#	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE~%#	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR~%#	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS~%#	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR~%#	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR~%#	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE~%#	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.~%#	~%#~%#	Adjust message provides a route for directly adjusting~%#	a state of the biomimetic model. There are two ways to~%#	specify an adjustment, selected independently for each~%#	adjustment channel.~%#~%#	1) Provide a target value in \"data\" and a \"gamma\" value~%#	between 0 and 1 to cause the state to approach the target:~%#~%#	(at 50Hz)~%#	state += gamma * (data - state)~%#~%#	2) Provide a delta value in \"data\" and set \"gamma\"~%#	to -1 to indicate this drive mode:~%#~%#	(at 50Hz)~%#	state += data~%#~%#	Understood values of gamma, therefore, are:~%#	   -1 : add \"data\" to state~%#	    0 : do nothing~%#	  0-1 : move state towards \"data\"~%#	    1 : instantly set state to \"data\"~%~%float32 data~%float32 gamma~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <adjust>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <adjust>))
  "Converts a ROS message object to a list"
  (cl:list 'adjust
    (cl:cons ':data (data msg))
    (cl:cons ':gamma (gamma msg))
))
