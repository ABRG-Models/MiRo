; Auto-generated. Do not edit!


(cl:in-package miro2_msg-msg)


;//! \htmlinclude push.msg.html

(cl:defclass <push> (roslisp-msg-protocol:ros-message)
  ((link
    :reader link
    :initarg :link
    :type cl:integer
    :initform 0)
   (flags
    :reader flags
    :initarg :flags
    :type cl:integer
    :initform 0)
   (pushpos
    :reader pushpos
    :initarg :pushpos
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (pushvec
    :reader pushvec
    :initarg :pushvec
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3)))
)

(cl:defclass push (<push>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <push>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'push)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name miro2_msg-msg:<push> is deprecated: use miro2_msg-msg:push instead.")))

(cl:ensure-generic-function 'link-val :lambda-list '(m))
(cl:defmethod link-val ((m <push>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader miro2_msg-msg:link-val is deprecated.  Use miro2_msg-msg:link instead.")
  (link m))

(cl:ensure-generic-function 'flags-val :lambda-list '(m))
(cl:defmethod flags-val ((m <push>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader miro2_msg-msg:flags-val is deprecated.  Use miro2_msg-msg:flags instead.")
  (flags m))

(cl:ensure-generic-function 'pushpos-val :lambda-list '(m))
(cl:defmethod pushpos-val ((m <push>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader miro2_msg-msg:pushpos-val is deprecated.  Use miro2_msg-msg:pushpos instead.")
  (pushpos m))

(cl:ensure-generic-function 'pushvec-val :lambda-list '(m))
(cl:defmethod pushvec-val ((m <push>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader miro2_msg-msg:pushvec-val is deprecated.  Use miro2_msg-msg:pushvec instead.")
  (pushvec m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <push>) ostream)
  "Serializes a message object of type '<push>"
  (cl:let* ((signed (cl:slot-value msg 'link)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'flags)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pushpos) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pushvec) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <push>) istream)
  "Deserializes a message object of type '<push>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'link) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'flags)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'flags)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'flags)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'flags)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pushpos) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pushvec) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<push>)))
  "Returns string type for a message object of type '<push>"
  "miro2_msg/push")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'push)))
  "Returns string type for a message object of type 'push"
  "miro2_msg/push")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<push>)))
  "Returns md5sum for a message object of type '<push>"
  "6271e5b7c8f54208b938d70e5eaafbe6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'push)))
  "Returns md5sum for a message object of type 'push"
  "6271e5b7c8f54208b938d70e5eaafbe6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<push>)))
  "Returns full string definition for message of type '<push>"
  (cl:format cl:nil "#	@section COPYRIGHT~%#	Copyright (C) 2019 Consequential Robotics Ltd~%#	~%#	@section AUTHOR~%#	Consequential Robotics http://consequentialrobotics.com~%#	~%#	@section LICENSE~%#	For a full copy of the license agreement, see LICENSE in the~%#	MDK root directory.~%#	~%#	Subject to the terms of this Agreement, Consequential~%#	Robotics grants to you a limited, non-exclusive, non-~%#	transferable license, without right to sub-license, to use~%#	MIRO Developer Kit in accordance with this Agreement and any~%#	other written agreement with Consequential Robotics.~%#	Consequential Robotics does not transfer the title of MIRO~%#	Developer Kit to you; the license granted to you is not a~%#	sale. This agreement is a binding legal agreement between~%#	Consequential Robotics and the purchasers or users of MIRO~%#	Developer Kit.~%#	~%#	THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY~%#	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE~%#	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR~%#	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS~%#	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR~%#	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR~%#	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE~%#	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.~%#~%#	Description... TODO.~%~%#	The identifier of the pushed link~%int32 link~%~%#	DOCLINK PUSH FLAGS~%#~%#	The values of these flags are defined in miro2.constants~%#~%#	PUSH_FLAG_IMPULSE~%#		Treat push as an impulse (in mm).~%#~%#	PUSH_FLAG_VELOCITY~%#		Treat push as a velocity (in mm/sec).~%#~%#	NB: If neither of the above flags is set, the push should~%#	not be actioned at all (this condition is used in server~%#	to indicate \"not pending\").~%#~%#	PUSH_FLAG_NO_TRANSLATION~%#		Zero out any resultant change in pose.dr (i.e. only~%#		turn on the spot).~%#~%#	PUSH_FLAG_NO_ROTATION~%#		Zero out any resultant change in pose.dtheta /and/~%#		in pose.dr (i.e. do not move wheels at all).~%#~%#	PUSH_FLAG_NO_NECK_MOVEMENT~%#		Zero out any resultant change in neck configuration;~%#		This flag is independent of NO_TRANSLATION/ROTATION.~%#~%#	PUSH_FLAG_WAIT~%#		Cause the push processor to wait for further pushes~%#		before publishing a velocity at its output. This is~%#		required if you want to pass multiple push streams.~%uint32 flags~%~%#	The pushed point, in the reference frame of the pushed link~%geometry_msgs/Point pushpos~%~%#	The push vector, in the reference frame of the pushed link~%geometry_msgs/Vector3 pushvec~%~%~%~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'push)))
  "Returns full string definition for message of type 'push"
  (cl:format cl:nil "#	@section COPYRIGHT~%#	Copyright (C) 2019 Consequential Robotics Ltd~%#	~%#	@section AUTHOR~%#	Consequential Robotics http://consequentialrobotics.com~%#	~%#	@section LICENSE~%#	For a full copy of the license agreement, see LICENSE in the~%#	MDK root directory.~%#	~%#	Subject to the terms of this Agreement, Consequential~%#	Robotics grants to you a limited, non-exclusive, non-~%#	transferable license, without right to sub-license, to use~%#	MIRO Developer Kit in accordance with this Agreement and any~%#	other written agreement with Consequential Robotics.~%#	Consequential Robotics does not transfer the title of MIRO~%#	Developer Kit to you; the license granted to you is not a~%#	sale. This agreement is a binding legal agreement between~%#	Consequential Robotics and the purchasers or users of MIRO~%#	Developer Kit.~%#	~%#	THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY~%#	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE~%#	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR~%#	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS~%#	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR~%#	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR~%#	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE~%#	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.~%#~%#	Description... TODO.~%~%#	The identifier of the pushed link~%int32 link~%~%#	DOCLINK PUSH FLAGS~%#~%#	The values of these flags are defined in miro2.constants~%#~%#	PUSH_FLAG_IMPULSE~%#		Treat push as an impulse (in mm).~%#~%#	PUSH_FLAG_VELOCITY~%#		Treat push as a velocity (in mm/sec).~%#~%#	NB: If neither of the above flags is set, the push should~%#	not be actioned at all (this condition is used in server~%#	to indicate \"not pending\").~%#~%#	PUSH_FLAG_NO_TRANSLATION~%#		Zero out any resultant change in pose.dr (i.e. only~%#		turn on the spot).~%#~%#	PUSH_FLAG_NO_ROTATION~%#		Zero out any resultant change in pose.dtheta /and/~%#		in pose.dr (i.e. do not move wheels at all).~%#~%#	PUSH_FLAG_NO_NECK_MOVEMENT~%#		Zero out any resultant change in neck configuration;~%#		This flag is independent of NO_TRANSLATION/ROTATION.~%#~%#	PUSH_FLAG_WAIT~%#		Cause the push processor to wait for further pushes~%#		before publishing a velocity at its output. This is~%#		required if you want to pass multiple push streams.~%uint32 flags~%~%#	The pushed point, in the reference frame of the pushed link~%geometry_msgs/Point pushpos~%~%#	The push vector, in the reference frame of the pushed link~%geometry_msgs/Vector3 pushvec~%~%~%~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <push>))
  (cl:+ 0
     4
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pushpos))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pushvec))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <push>))
  "Converts a ROS message object to a list"
  (cl:list 'push
    (cl:cons ':link (link msg))
    (cl:cons ':flags (flags msg))
    (cl:cons ':pushpos (pushpos msg))
    (cl:cons ':pushvec (pushvec msg))
))
