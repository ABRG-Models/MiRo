; Auto-generated. Do not edit!


(cl:in-package miro2_msg-msg)


;//! \htmlinclude affect_adjust.msg.html

(cl:defclass <affect_adjust> (roslisp-msg-protocol:ros-message)
  ((valence
    :reader valence
    :initarg :valence
    :type miro2_msg-msg:adjust
    :initform (cl:make-instance 'miro2_msg-msg:adjust))
   (arousal
    :reader arousal
    :initarg :arousal
    :type miro2_msg-msg:adjust
    :initform (cl:make-instance 'miro2_msg-msg:adjust)))
)

(cl:defclass affect_adjust (<affect_adjust>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <affect_adjust>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'affect_adjust)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name miro2_msg-msg:<affect_adjust> is deprecated: use miro2_msg-msg:affect_adjust instead.")))

(cl:ensure-generic-function 'valence-val :lambda-list '(m))
(cl:defmethod valence-val ((m <affect_adjust>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader miro2_msg-msg:valence-val is deprecated.  Use miro2_msg-msg:valence instead.")
  (valence m))

(cl:ensure-generic-function 'arousal-val :lambda-list '(m))
(cl:defmethod arousal-val ((m <affect_adjust>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader miro2_msg-msg:arousal-val is deprecated.  Use miro2_msg-msg:arousal instead.")
  (arousal m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <affect_adjust>) ostream)
  "Serializes a message object of type '<affect_adjust>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'valence) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'arousal) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <affect_adjust>) istream)
  "Deserializes a message object of type '<affect_adjust>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'valence) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'arousal) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<affect_adjust>)))
  "Returns string type for a message object of type '<affect_adjust>"
  "miro2_msg/affect_adjust")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'affect_adjust)))
  "Returns string type for a message object of type 'affect_adjust"
  "miro2_msg/affect_adjust")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<affect_adjust>)))
  "Returns md5sum for a message object of type '<affect_adjust>"
  "80f7164a0f33b02beb56e574f19360cc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'affect_adjust)))
  "Returns md5sum for a message object of type 'affect_adjust"
  "80f7164a0f33b02beb56e574f19360cc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<affect_adjust>)))
  "Returns full string definition for message of type '<affect_adjust>"
  (cl:format cl:nil "#	@section COPYRIGHT~%#	Copyright (C) 2019 Consequential Robotics Ltd~%#	~%#	@section AUTHOR~%#	Consequential Robotics http://consequentialrobotics.com~%#	~%#	@section LICENSE~%#	For a full copy of the license agreement, and a complete~%#	definition of \"The Software\", see LICENSE in the MDK root~%#	directory.~%#	~%#	Subject to the terms of this Agreement, Consequential~%#	Robotics grants to you a limited, non-exclusive, non-~%#	transferable license, without right to sub-license, to use~%#	\"The Software\" in accordance with this Agreement and any~%#	other written agreement with Consequential Robotics.~%#	Consequential Robotics does not transfer the title of \"The~%#	Software\" to you; the license granted to you is not a sale.~%#	This agreement is a binding legal agreement between~%#	Consequential Robotics and the purchasers or users of \"The~%#	Software\".~%#	~%#	THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY~%#	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE~%#	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR~%#	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS~%#	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR~%#	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR~%#	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE~%#	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.~%#	~%~%adjust valence~%adjust arousal~%~%~%================================================================================~%MSG: miro2_msg/adjust~%#	@section COPYRIGHT~%#	Copyright (C) 2019 Consequential Robotics Ltd~%#	~%#	@section AUTHOR~%#	Consequential Robotics http://consequentialrobotics.com~%#	~%#	@section LICENSE~%#	For a full copy of the license agreement, and a complete~%#	definition of \"The Software\", see LICENSE in the MDK root~%#	directory.~%#	~%#	Subject to the terms of this Agreement, Consequential~%#	Robotics grants to you a limited, non-exclusive, non-~%#	transferable license, without right to sub-license, to use~%#	\"The Software\" in accordance with this Agreement and any~%#	other written agreement with Consequential Robotics.~%#	Consequential Robotics does not transfer the title of \"The~%#	Software\" to you; the license granted to you is not a sale.~%#	This agreement is a binding legal agreement between~%#	Consequential Robotics and the purchasers or users of \"The~%#	Software\".~%#	~%#	THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY~%#	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE~%#	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR~%#	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS~%#	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR~%#	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR~%#	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE~%#	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.~%#	~%#~%#	Adjust message provides a route for directly adjusting~%#	a state of the biomimetic model. There are two ways to~%#	specify an adjustment, selected independently for each~%#	adjustment channel.~%#~%#	1) Provide a target value in \"data\" and a \"gamma\" value~%#	between 0 and 1 to cause the state to approach the target:~%#~%#	(at 50Hz)~%#	state += gamma * (data - state)~%#~%#	2) Provide a delta value in \"data\" and set \"gamma\"~%#	to -1 to indicate this drive mode:~%#~%#	(at 50Hz)~%#	state += data~%#~%#	Understood values of gamma, therefore, are:~%#	   -1 : add \"data\" to state~%#	    0 : do nothing~%#	  0-1 : move state towards \"data\"~%#	    1 : instantly set state to \"data\"~%~%float32 data~%float32 gamma~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'affect_adjust)))
  "Returns full string definition for message of type 'affect_adjust"
  (cl:format cl:nil "#	@section COPYRIGHT~%#	Copyright (C) 2019 Consequential Robotics Ltd~%#	~%#	@section AUTHOR~%#	Consequential Robotics http://consequentialrobotics.com~%#	~%#	@section LICENSE~%#	For a full copy of the license agreement, and a complete~%#	definition of \"The Software\", see LICENSE in the MDK root~%#	directory.~%#	~%#	Subject to the terms of this Agreement, Consequential~%#	Robotics grants to you a limited, non-exclusive, non-~%#	transferable license, without right to sub-license, to use~%#	\"The Software\" in accordance with this Agreement and any~%#	other written agreement with Consequential Robotics.~%#	Consequential Robotics does not transfer the title of \"The~%#	Software\" to you; the license granted to you is not a sale.~%#	This agreement is a binding legal agreement between~%#	Consequential Robotics and the purchasers or users of \"The~%#	Software\".~%#	~%#	THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY~%#	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE~%#	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR~%#	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS~%#	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR~%#	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR~%#	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE~%#	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.~%#	~%~%adjust valence~%adjust arousal~%~%~%================================================================================~%MSG: miro2_msg/adjust~%#	@section COPYRIGHT~%#	Copyright (C) 2019 Consequential Robotics Ltd~%#	~%#	@section AUTHOR~%#	Consequential Robotics http://consequentialrobotics.com~%#	~%#	@section LICENSE~%#	For a full copy of the license agreement, and a complete~%#	definition of \"The Software\", see LICENSE in the MDK root~%#	directory.~%#	~%#	Subject to the terms of this Agreement, Consequential~%#	Robotics grants to you a limited, non-exclusive, non-~%#	transferable license, without right to sub-license, to use~%#	\"The Software\" in accordance with this Agreement and any~%#	other written agreement with Consequential Robotics.~%#	Consequential Robotics does not transfer the title of \"The~%#	Software\" to you; the license granted to you is not a sale.~%#	This agreement is a binding legal agreement between~%#	Consequential Robotics and the purchasers or users of \"The~%#	Software\".~%#	~%#	THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY~%#	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE~%#	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR~%#	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS~%#	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR~%#	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR~%#	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE~%#	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.~%#	~%#~%#	Adjust message provides a route for directly adjusting~%#	a state of the biomimetic model. There are two ways to~%#	specify an adjustment, selected independently for each~%#	adjustment channel.~%#~%#	1) Provide a target value in \"data\" and a \"gamma\" value~%#	between 0 and 1 to cause the state to approach the target:~%#~%#	(at 50Hz)~%#	state += gamma * (data - state)~%#~%#	2) Provide a delta value in \"data\" and set \"gamma\"~%#	to -1 to indicate this drive mode:~%#~%#	(at 50Hz)~%#	state += data~%#~%#	Understood values of gamma, therefore, are:~%#	   -1 : add \"data\" to state~%#	    0 : do nothing~%#	  0-1 : move state towards \"data\"~%#	    1 : instantly set state to \"data\"~%~%float32 data~%float32 gamma~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <affect_adjust>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'valence))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'arousal))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <affect_adjust>))
  "Converts a ROS message object to a list"
  (cl:list 'affect_adjust
    (cl:cons ':valence (valence msg))
    (cl:cons ':arousal (arousal msg))
))
