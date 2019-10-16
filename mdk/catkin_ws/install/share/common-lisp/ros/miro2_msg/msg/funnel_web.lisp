; Auto-generated. Do not edit!


(cl:in-package miro2_msg-msg)


;//! \htmlinclude funnel_web.msg.html

(cl:defclass <funnel_web> (roslisp-msg-protocol:ros-message)
  ((cliff
    :reader cliff
    :initarg :cliff
    :type (cl:vector cl:float)
   :initform (cl:make-array 2 :element-type 'cl:float :initial-element 0.0))
   (light
    :reader light
    :initarg :light
    :type (cl:vector cl:float)
   :initform (cl:make-array 4 :element-type 'cl:float :initial-element 0.0))
   (sonar
    :reader sonar
    :initarg :sonar
    :type cl:float
    :initform 0.0)
   (touch
    :reader touch
    :initarg :touch
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 2 :element-type 'cl:fixnum :initial-element 0))
   (illum
    :reader illum
    :initarg :illum
    :type (cl:vector cl:integer)
   :initform (cl:make-array 6 :element-type 'cl:integer :initial-element 0))
   (audio_level
    :reader audio_level
    :initarg :audio_level
    :type cl:float
    :initform 0.0)
   (tone
    :reader tone
    :initarg :tone
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 3 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass funnel_web (<funnel_web>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <funnel_web>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'funnel_web)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name miro2_msg-msg:<funnel_web> is deprecated: use miro2_msg-msg:funnel_web instead.")))

(cl:ensure-generic-function 'cliff-val :lambda-list '(m))
(cl:defmethod cliff-val ((m <funnel_web>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader miro2_msg-msg:cliff-val is deprecated.  Use miro2_msg-msg:cliff instead.")
  (cliff m))

(cl:ensure-generic-function 'light-val :lambda-list '(m))
(cl:defmethod light-val ((m <funnel_web>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader miro2_msg-msg:light-val is deprecated.  Use miro2_msg-msg:light instead.")
  (light m))

(cl:ensure-generic-function 'sonar-val :lambda-list '(m))
(cl:defmethod sonar-val ((m <funnel_web>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader miro2_msg-msg:sonar-val is deprecated.  Use miro2_msg-msg:sonar instead.")
  (sonar m))

(cl:ensure-generic-function 'touch-val :lambda-list '(m))
(cl:defmethod touch-val ((m <funnel_web>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader miro2_msg-msg:touch-val is deprecated.  Use miro2_msg-msg:touch instead.")
  (touch m))

(cl:ensure-generic-function 'illum-val :lambda-list '(m))
(cl:defmethod illum-val ((m <funnel_web>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader miro2_msg-msg:illum-val is deprecated.  Use miro2_msg-msg:illum instead.")
  (illum m))

(cl:ensure-generic-function 'audio_level-val :lambda-list '(m))
(cl:defmethod audio_level-val ((m <funnel_web>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader miro2_msg-msg:audio_level-val is deprecated.  Use miro2_msg-msg:audio_level instead.")
  (audio_level m))

(cl:ensure-generic-function 'tone-val :lambda-list '(m))
(cl:defmethod tone-val ((m <funnel_web>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader miro2_msg-msg:tone-val is deprecated.  Use miro2_msg-msg:tone instead.")
  (tone m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <funnel_web>) ostream)
  "Serializes a message object of type '<funnel_web>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'cliff))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'light))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'sonar))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'touch))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) ele) ostream))
   (cl:slot-value msg 'illum))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'audio_level))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'tone))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <funnel_web>) istream)
  "Deserializes a message object of type '<funnel_web>"
  (cl:setf (cl:slot-value msg 'cliff) (cl:make-array 2))
  (cl:let ((vals (cl:slot-value msg 'cliff)))
    (cl:dotimes (i 2)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'light) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'light)))
    (cl:dotimes (i 4)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'sonar) (roslisp-utils:decode-single-float-bits bits)))
  (cl:setf (cl:slot-value msg 'touch) (cl:make-array 2))
  (cl:let ((vals (cl:slot-value msg 'touch)))
    (cl:dotimes (i 2)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))))
  (cl:setf (cl:slot-value msg 'illum) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'illum)))
    (cl:dotimes (i 6)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:aref vals i)) (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'audio_level) (roslisp-utils:decode-single-float-bits bits)))
  (cl:setf (cl:slot-value msg 'tone) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'tone)))
    (cl:dotimes (i 3)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<funnel_web>)))
  "Returns string type for a message object of type '<funnel_web>"
  "miro2_msg/funnel_web")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'funnel_web)))
  "Returns string type for a message object of type 'funnel_web"
  "miro2_msg/funnel_web")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<funnel_web>)))
  "Returns md5sum for a message object of type '<funnel_web>"
  "d76c86fb9327536412ade3373c6dd88e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'funnel_web)))
  "Returns md5sum for a message object of type 'funnel_web"
  "d76c86fb9327536412ade3373c6dd88e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<funnel_web>)))
  "Returns full string definition for message of type '<funnel_web>"
  (cl:format cl:nil "#	@section COPYRIGHT~%#	Copyright (C) 2019 Consequential Robotics Ltd~%#	~%#	@section AUTHOR~%#	Consequential Robotics http://consequentialrobotics.com~%#	~%#	@section LICENSE~%#	For a full copy of the license agreement, and a complete~%#	definition of \"The Software\", see LICENSE in the MDK root~%#	directory.~%#	~%#	Subject to the terms of this Agreement, Consequential~%#	Robotics grants to you a limited, non-exclusive, non-~%#	transferable license, without right to sub-license, to use~%#	\"The Software\" in accordance with this Agreement and any~%#	other written agreement with Consequential Robotics.~%#	Consequential Robotics does not transfer the title of \"The~%#	Software\" to you; the license granted to you is not a sale.~%#	This agreement is a binding legal agreement between~%#	Consequential Robotics and the purchasers or users of \"The~%#	Software\".~%#	~%#	THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY~%#	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE~%#	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR~%#	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS~%#	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR~%#	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR~%#	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE~%#	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.~%#	~%#~%#	This message packages several messages for simple~%#	delivery to the web client.~%~%~%~%float32[2] cliff~%float32[4] light~%float32 sonar~%uint16[2] touch~%uint32[6] illum~%float32 audio_level~%uint16[3] tone~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'funnel_web)))
  "Returns full string definition for message of type 'funnel_web"
  (cl:format cl:nil "#	@section COPYRIGHT~%#	Copyright (C) 2019 Consequential Robotics Ltd~%#	~%#	@section AUTHOR~%#	Consequential Robotics http://consequentialrobotics.com~%#	~%#	@section LICENSE~%#	For a full copy of the license agreement, and a complete~%#	definition of \"The Software\", see LICENSE in the MDK root~%#	directory.~%#	~%#	Subject to the terms of this Agreement, Consequential~%#	Robotics grants to you a limited, non-exclusive, non-~%#	transferable license, without right to sub-license, to use~%#	\"The Software\" in accordance with this Agreement and any~%#	other written agreement with Consequential Robotics.~%#	Consequential Robotics does not transfer the title of \"The~%#	Software\" to you; the license granted to you is not a sale.~%#	This agreement is a binding legal agreement between~%#	Consequential Robotics and the purchasers or users of \"The~%#	Software\".~%#	~%#	THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY~%#	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE~%#	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR~%#	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS~%#	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR~%#	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR~%#	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE~%#	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.~%#	~%#~%#	This message packages several messages for simple~%#	delivery to the web client.~%~%~%~%float32[2] cliff~%float32[4] light~%float32 sonar~%uint16[2] touch~%uint32[6] illum~%float32 audio_level~%uint16[3] tone~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <funnel_web>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'cliff) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'light) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'touch) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'illum) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'tone) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <funnel_web>))
  "Converts a ROS message object to a list"
  (cl:list 'funnel_web
    (cl:cons ':cliff (cliff msg))
    (cl:cons ':light (light msg))
    (cl:cons ':sonar (sonar msg))
    (cl:cons ':touch (touch msg))
    (cl:cons ':illum (illum msg))
    (cl:cons ':audio_level (audio_level msg))
    (cl:cons ':tone (tone msg))
))
