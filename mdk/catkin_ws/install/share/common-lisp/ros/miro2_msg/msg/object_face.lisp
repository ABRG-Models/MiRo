; Auto-generated. Do not edit!


(cl:in-package miro2_msg-msg)


;//! \htmlinclude object_face.msg.html

(cl:defclass <object_face> (roslisp-msg-protocol:ros-message)
  ((conf
    :reader conf
    :initarg :conf
    :type cl:float
    :initform 0.0)
   (corner
    :reader corner
    :initarg :corner
    :type (cl:vector cl:float)
   :initform (cl:make-array 2 :element-type 'cl:float :initial-element 0.0))
   (size
    :reader size
    :initarg :size
    :type (cl:vector cl:float)
   :initform (cl:make-array 2 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass object_face (<object_face>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <object_face>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'object_face)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name miro2_msg-msg:<object_face> is deprecated: use miro2_msg-msg:object_face instead.")))

(cl:ensure-generic-function 'conf-val :lambda-list '(m))
(cl:defmethod conf-val ((m <object_face>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader miro2_msg-msg:conf-val is deprecated.  Use miro2_msg-msg:conf instead.")
  (conf m))

(cl:ensure-generic-function 'corner-val :lambda-list '(m))
(cl:defmethod corner-val ((m <object_face>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader miro2_msg-msg:corner-val is deprecated.  Use miro2_msg-msg:corner instead.")
  (corner m))

(cl:ensure-generic-function 'size-val :lambda-list '(m))
(cl:defmethod size-val ((m <object_face>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader miro2_msg-msg:size-val is deprecated.  Use miro2_msg-msg:size instead.")
  (size m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <object_face>) ostream)
  "Serializes a message object of type '<object_face>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'conf))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'corner))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'size))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <object_face>) istream)
  "Deserializes a message object of type '<object_face>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'conf) (roslisp-utils:decode-single-float-bits bits)))
  (cl:setf (cl:slot-value msg 'corner) (cl:make-array 2))
  (cl:let ((vals (cl:slot-value msg 'corner)))
    (cl:dotimes (i 2)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'size) (cl:make-array 2))
  (cl:let ((vals (cl:slot-value msg 'size)))
    (cl:dotimes (i 2)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<object_face>)))
  "Returns string type for a message object of type '<object_face>"
  "miro2_msg/object_face")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'object_face)))
  "Returns string type for a message object of type 'object_face"
  "miro2_msg/object_face")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<object_face>)))
  "Returns md5sum for a message object of type '<object_face>"
  "0608d5dd7e088755bafb29283da99f24")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'object_face)))
  "Returns md5sum for a message object of type 'object_face"
  "0608d5dd7e088755bafb29283da99f24")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<object_face>)))
  "Returns full string definition for message of type '<object_face>"
  (cl:format cl:nil "#	@section COPYRIGHT~%#	Copyright (C) 2019 Consequential Robotics Ltd~%#	~%#	@section AUTHOR~%#	Consequential Robotics http://consequentialrobotics.com~%#	~%#	@section LICENSE~%#	For a full copy of the license agreement, and a complete~%#	definition of \"The Software\", see LICENSE in the MDK root~%#	directory.~%#	~%#	Subject to the terms of this Agreement, Consequential~%#	Robotics grants to you a limited, non-exclusive, non-~%#	transferable license, without right to sub-license, to use~%#	\"The Software\" in accordance with this Agreement and any~%#	other written agreement with Consequential Robotics.~%#	Consequential Robotics does not transfer the title of \"The~%#	Software\" to you; the license granted to you is not a sale.~%#	This agreement is a binding legal agreement between~%#	Consequential Robotics and the purchasers or users of \"The~%#	Software\".~%#	~%#	THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY~%#	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE~%#	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR~%#	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS~%#	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR~%#	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR~%#	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE~%#	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.~%#	~%~%float32 conf~%float32[2] corner~%float32[2] size~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'object_face)))
  "Returns full string definition for message of type 'object_face"
  (cl:format cl:nil "#	@section COPYRIGHT~%#	Copyright (C) 2019 Consequential Robotics Ltd~%#	~%#	@section AUTHOR~%#	Consequential Robotics http://consequentialrobotics.com~%#	~%#	@section LICENSE~%#	For a full copy of the license agreement, and a complete~%#	definition of \"The Software\", see LICENSE in the MDK root~%#	directory.~%#	~%#	Subject to the terms of this Agreement, Consequential~%#	Robotics grants to you a limited, non-exclusive, non-~%#	transferable license, without right to sub-license, to use~%#	\"The Software\" in accordance with this Agreement and any~%#	other written agreement with Consequential Robotics.~%#	Consequential Robotics does not transfer the title of \"The~%#	Software\" to you; the license granted to you is not a sale.~%#	This agreement is a binding legal agreement between~%#	Consequential Robotics and the purchasers or users of \"The~%#	Software\".~%#	~%#	THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY~%#	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE~%#	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR~%#	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS~%#	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR~%#	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR~%#	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE~%#	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.~%#	~%~%float32 conf~%float32[2] corner~%float32[2] size~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <object_face>))
  (cl:+ 0
     4
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'corner) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'size) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <object_face>))
  "Converts a ROS message object to a list"
  (cl:list 'object_face
    (cl:cons ':conf (conf msg))
    (cl:cons ':corner (corner msg))
    (cl:cons ':size (size msg))
))
