; Auto-generated. Do not edit!


(cl:in-package miro2_msg-msg)


;//! \htmlinclude priority_peak.msg.html

(cl:defclass <priority_peak> (roslisp-msg-protocol:ros-message)
  ((stream_index
    :reader stream_index
    :initarg :stream_index
    :type cl:integer
    :initform 0)
   (loc_d
    :reader loc_d
    :initarg :loc_d
    :type (cl:vector cl:float)
   :initform (cl:make-array 2 :element-type 'cl:float :initial-element 0.0))
   (height
    :reader height
    :initarg :height
    :type cl:float
    :initform 0.0)
   (size
    :reader size
    :initarg :size
    :type cl:float
    :initform 0.0)
   (azim
    :reader azim
    :initarg :azim
    :type cl:float
    :initform 0.0)
   (elev
    :reader elev
    :initarg :elev
    :type cl:float
    :initform 0.0)
   (size_norm
    :reader size_norm
    :initarg :size_norm
    :type cl:float
    :initform 0.0)
   (volume
    :reader volume
    :initarg :volume
    :type cl:float
    :initform 0.0)
   (range
    :reader range
    :initarg :range
    :type cl:float
    :initform 0.0)
   (actioned
    :reader actioned
    :initarg :actioned
    :type cl:integer
    :initform 0))
)

(cl:defclass priority_peak (<priority_peak>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <priority_peak>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'priority_peak)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name miro2_msg-msg:<priority_peak> is deprecated: use miro2_msg-msg:priority_peak instead.")))

(cl:ensure-generic-function 'stream_index-val :lambda-list '(m))
(cl:defmethod stream_index-val ((m <priority_peak>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader miro2_msg-msg:stream_index-val is deprecated.  Use miro2_msg-msg:stream_index instead.")
  (stream_index m))

(cl:ensure-generic-function 'loc_d-val :lambda-list '(m))
(cl:defmethod loc_d-val ((m <priority_peak>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader miro2_msg-msg:loc_d-val is deprecated.  Use miro2_msg-msg:loc_d instead.")
  (loc_d m))

(cl:ensure-generic-function 'height-val :lambda-list '(m))
(cl:defmethod height-val ((m <priority_peak>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader miro2_msg-msg:height-val is deprecated.  Use miro2_msg-msg:height instead.")
  (height m))

(cl:ensure-generic-function 'size-val :lambda-list '(m))
(cl:defmethod size-val ((m <priority_peak>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader miro2_msg-msg:size-val is deprecated.  Use miro2_msg-msg:size instead.")
  (size m))

(cl:ensure-generic-function 'azim-val :lambda-list '(m))
(cl:defmethod azim-val ((m <priority_peak>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader miro2_msg-msg:azim-val is deprecated.  Use miro2_msg-msg:azim instead.")
  (azim m))

(cl:ensure-generic-function 'elev-val :lambda-list '(m))
(cl:defmethod elev-val ((m <priority_peak>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader miro2_msg-msg:elev-val is deprecated.  Use miro2_msg-msg:elev instead.")
  (elev m))

(cl:ensure-generic-function 'size_norm-val :lambda-list '(m))
(cl:defmethod size_norm-val ((m <priority_peak>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader miro2_msg-msg:size_norm-val is deprecated.  Use miro2_msg-msg:size_norm instead.")
  (size_norm m))

(cl:ensure-generic-function 'volume-val :lambda-list '(m))
(cl:defmethod volume-val ((m <priority_peak>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader miro2_msg-msg:volume-val is deprecated.  Use miro2_msg-msg:volume instead.")
  (volume m))

(cl:ensure-generic-function 'range-val :lambda-list '(m))
(cl:defmethod range-val ((m <priority_peak>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader miro2_msg-msg:range-val is deprecated.  Use miro2_msg-msg:range instead.")
  (range m))

(cl:ensure-generic-function 'actioned-val :lambda-list '(m))
(cl:defmethod actioned-val ((m <priority_peak>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader miro2_msg-msg:actioned-val is deprecated.  Use miro2_msg-msg:actioned instead.")
  (actioned m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <priority_peak>) ostream)
  "Serializes a message object of type '<priority_peak>"
  (cl:let* ((signed (cl:slot-value msg 'stream_index)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'loc_d))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'height))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'size))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'azim))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'elev))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'size_norm))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'volume))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'range))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'actioned)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <priority_peak>) istream)
  "Deserializes a message object of type '<priority_peak>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'stream_index) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:setf (cl:slot-value msg 'loc_d) (cl:make-array 2))
  (cl:let ((vals (cl:slot-value msg 'loc_d)))
    (cl:dotimes (i 2)
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
    (cl:setf (cl:slot-value msg 'height) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'size) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'azim) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'elev) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'size_norm) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'volume) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'range) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'actioned) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<priority_peak>)))
  "Returns string type for a message object of type '<priority_peak>"
  "miro2_msg/priority_peak")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'priority_peak)))
  "Returns string type for a message object of type 'priority_peak"
  "miro2_msg/priority_peak")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<priority_peak>)))
  "Returns md5sum for a message object of type '<priority_peak>"
  "d2924675ffe77da16f66ce7eb1cbeb6d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'priority_peak)))
  "Returns md5sum for a message object of type 'priority_peak"
  "d2924675ffe77da16f66ce7eb1cbeb6d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<priority_peak>)))
  "Returns full string definition for message of type '<priority_peak>"
  (cl:format cl:nil "#	@section COPYRIGHT~%#	Copyright (C) 2019 Consequential Robotics Ltd~%#	~%#	@section AUTHOR~%#	Consequential Robotics http://consequentialrobotics.com~%#	~%#	@section LICENSE~%#	For a full copy of the license agreement, and a complete~%#	definition of \"The Software\", see LICENSE in the MDK root~%#	directory.~%#	~%#	Subject to the terms of this Agreement, Consequential~%#	Robotics grants to you a limited, non-exclusive, non-~%#	transferable license, without right to sub-license, to use~%#	\"The Software\" in accordance with this Agreement and any~%#	other written agreement with Consequential Robotics.~%#	Consequential Robotics does not transfer the title of \"The~%#	Software\" to you; the license granted to you is not a sale.~%#	This agreement is a binding legal agreement between~%#	Consequential Robotics and the purchasers or users of \"The~%#	Software\".~%#	~%#	THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY~%#	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE~%#	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR~%#	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS~%#	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR~%#	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR~%#	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE~%#	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.~%#	~%~%int32 stream_index~%float32[2] loc_d~%float32 height~%float32 size~%float32 azim~%float32 elev~%~%float32 size_norm~%float32 volume~%float32 range~%~%int32 actioned~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'priority_peak)))
  "Returns full string definition for message of type 'priority_peak"
  (cl:format cl:nil "#	@section COPYRIGHT~%#	Copyright (C) 2019 Consequential Robotics Ltd~%#	~%#	@section AUTHOR~%#	Consequential Robotics http://consequentialrobotics.com~%#	~%#	@section LICENSE~%#	For a full copy of the license agreement, and a complete~%#	definition of \"The Software\", see LICENSE in the MDK root~%#	directory.~%#	~%#	Subject to the terms of this Agreement, Consequential~%#	Robotics grants to you a limited, non-exclusive, non-~%#	transferable license, without right to sub-license, to use~%#	\"The Software\" in accordance with this Agreement and any~%#	other written agreement with Consequential Robotics.~%#	Consequential Robotics does not transfer the title of \"The~%#	Software\" to you; the license granted to you is not a sale.~%#	This agreement is a binding legal agreement between~%#	Consequential Robotics and the purchasers or users of \"The~%#	Software\".~%#	~%#	THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY~%#	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE~%#	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR~%#	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS~%#	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR~%#	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR~%#	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE~%#	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.~%#	~%~%int32 stream_index~%float32[2] loc_d~%float32 height~%float32 size~%float32 azim~%float32 elev~%~%float32 size_norm~%float32 volume~%float32 range~%~%int32 actioned~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <priority_peak>))
  (cl:+ 0
     4
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'loc_d) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <priority_peak>))
  "Converts a ROS message object to a list"
  (cl:list 'priority_peak
    (cl:cons ':stream_index (stream_index msg))
    (cl:cons ':loc_d (loc_d msg))
    (cl:cons ':height (height msg))
    (cl:cons ':size (size msg))
    (cl:cons ':azim (azim msg))
    (cl:cons ':elev (elev msg))
    (cl:cons ':size_norm (size_norm msg))
    (cl:cons ':volume (volume msg))
    (cl:cons ':range (range msg))
    (cl:cons ':actioned (actioned msg))
))
