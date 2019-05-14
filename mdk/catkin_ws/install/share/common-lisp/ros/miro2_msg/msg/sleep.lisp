; Auto-generated. Do not edit!


(cl:in-package miro2_msg-msg)


;//! \htmlinclude sleep.msg.html

(cl:defclass <sleep> (roslisp-msg-protocol:ros-message)
  ((wakefulness
    :reader wakefulness
    :initarg :wakefulness
    :type cl:float
    :initform 0.0)
   (pressure
    :reader pressure
    :initarg :pressure
    :type cl:float
    :initform 0.0))
)

(cl:defclass sleep (<sleep>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <sleep>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'sleep)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name miro2_msg-msg:<sleep> is deprecated: use miro2_msg-msg:sleep instead.")))

(cl:ensure-generic-function 'wakefulness-val :lambda-list '(m))
(cl:defmethod wakefulness-val ((m <sleep>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader miro2_msg-msg:wakefulness-val is deprecated.  Use miro2_msg-msg:wakefulness instead.")
  (wakefulness m))

(cl:ensure-generic-function 'pressure-val :lambda-list '(m))
(cl:defmethod pressure-val ((m <sleep>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader miro2_msg-msg:pressure-val is deprecated.  Use miro2_msg-msg:pressure instead.")
  (pressure m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <sleep>) ostream)
  "Serializes a message object of type '<sleep>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'wakefulness))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pressure))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <sleep>) istream)
  "Deserializes a message object of type '<sleep>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'wakefulness) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pressure) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<sleep>)))
  "Returns string type for a message object of type '<sleep>"
  "miro2_msg/sleep")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'sleep)))
  "Returns string type for a message object of type 'sleep"
  "miro2_msg/sleep")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<sleep>)))
  "Returns md5sum for a message object of type '<sleep>"
  "9ae301b8349f95e1749450e5431eef09")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'sleep)))
  "Returns md5sum for a message object of type 'sleep"
  "9ae301b8349f95e1749450e5431eef09")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<sleep>)))
  "Returns full string definition for message of type '<sleep>"
  (cl:format cl:nil "#	@section COPYRIGHT~%#	Copyright (C) 2019 Consequential Robotics Ltd~%#	~%#	@section AUTHOR~%#	Consequential Robotics http://consequentialrobotics.com~%#	~%#	@section LICENSE~%#	For a full copy of the license agreement, see LICENSE in the~%#	MDK root directory.~%#	~%#	Subject to the terms of this Agreement, Consequential~%#	Robotics grants to you a limited, non-exclusive, non-~%#	transferable license, without right to sub-license, to use~%#	MIRO Developer Kit in accordance with this Agreement and any~%#	other written agreement with Consequential Robotics.~%#	Consequential Robotics does not transfer the title of MIRO~%#	Developer Kit to you; the license granted to you is not a~%#	sale. This agreement is a binding legal agreement between~%#	Consequential Robotics and the purchasers or users of MIRO~%#	Developer Kit.~%#	~%#	THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY~%#	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE~%#	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR~%#	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS~%#	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR~%#	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR~%#	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE~%#	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.~%#~%#	The \"sleep\" state is two-dimensional, encoding \"wakefulness\"~%#	(0.0 to 1.0, what it sounds like) and \"pressure\" (0.0 to 1.0,~%#	tendency to move towards reduced wakefulness). The two states~%#	evolve together to implement a relaxation oscillator.~%~%float32 wakefulness~%float32 pressure~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'sleep)))
  "Returns full string definition for message of type 'sleep"
  (cl:format cl:nil "#	@section COPYRIGHT~%#	Copyright (C) 2019 Consequential Robotics Ltd~%#	~%#	@section AUTHOR~%#	Consequential Robotics http://consequentialrobotics.com~%#	~%#	@section LICENSE~%#	For a full copy of the license agreement, see LICENSE in the~%#	MDK root directory.~%#	~%#	Subject to the terms of this Agreement, Consequential~%#	Robotics grants to you a limited, non-exclusive, non-~%#	transferable license, without right to sub-license, to use~%#	MIRO Developer Kit in accordance with this Agreement and any~%#	other written agreement with Consequential Robotics.~%#	Consequential Robotics does not transfer the title of MIRO~%#	Developer Kit to you; the license granted to you is not a~%#	sale. This agreement is a binding legal agreement between~%#	Consequential Robotics and the purchasers or users of MIRO~%#	Developer Kit.~%#	~%#	THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY~%#	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE~%#	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR~%#	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS~%#	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR~%#	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR~%#	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE~%#	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.~%#~%#	The \"sleep\" state is two-dimensional, encoding \"wakefulness\"~%#	(0.0 to 1.0, what it sounds like) and \"pressure\" (0.0 to 1.0,~%#	tendency to move towards reduced wakefulness). The two states~%#	evolve together to implement a relaxation oscillator.~%~%float32 wakefulness~%float32 pressure~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <sleep>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <sleep>))
  "Converts a ROS message object to a list"
  (cl:list 'sleep
    (cl:cons ':wakefulness (wakefulness msg))
    (cl:cons ':pressure (pressure msg))
))
