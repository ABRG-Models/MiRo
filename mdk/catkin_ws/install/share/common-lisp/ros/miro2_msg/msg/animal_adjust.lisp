; Auto-generated. Do not edit!


(cl:in-package miro2_msg-msg)


;//! \htmlinclude animal_adjust.msg.html

(cl:defclass <animal_adjust> (roslisp-msg-protocol:ros-message)
  ((mood
    :reader mood
    :initarg :mood
    :type miro2_msg-msg:affect_adjust
    :initform (cl:make-instance 'miro2_msg-msg:affect_adjust))
   (emotion
    :reader emotion
    :initarg :emotion
    :type miro2_msg-msg:affect_adjust
    :initform (cl:make-instance 'miro2_msg-msg:affect_adjust))
   (sleep
    :reader sleep
    :initarg :sleep
    :type miro2_msg-msg:sleep_adjust
    :initform (cl:make-instance 'miro2_msg-msg:sleep_adjust)))
)

(cl:defclass animal_adjust (<animal_adjust>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <animal_adjust>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'animal_adjust)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name miro2_msg-msg:<animal_adjust> is deprecated: use miro2_msg-msg:animal_adjust instead.")))

(cl:ensure-generic-function 'mood-val :lambda-list '(m))
(cl:defmethod mood-val ((m <animal_adjust>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader miro2_msg-msg:mood-val is deprecated.  Use miro2_msg-msg:mood instead.")
  (mood m))

(cl:ensure-generic-function 'emotion-val :lambda-list '(m))
(cl:defmethod emotion-val ((m <animal_adjust>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader miro2_msg-msg:emotion-val is deprecated.  Use miro2_msg-msg:emotion instead.")
  (emotion m))

(cl:ensure-generic-function 'sleep-val :lambda-list '(m))
(cl:defmethod sleep-val ((m <animal_adjust>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader miro2_msg-msg:sleep-val is deprecated.  Use miro2_msg-msg:sleep instead.")
  (sleep m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <animal_adjust>) ostream)
  "Serializes a message object of type '<animal_adjust>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'mood) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'emotion) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'sleep) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <animal_adjust>) istream)
  "Deserializes a message object of type '<animal_adjust>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'mood) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'emotion) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'sleep) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<animal_adjust>)))
  "Returns string type for a message object of type '<animal_adjust>"
  "miro2_msg/animal_adjust")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'animal_adjust)))
  "Returns string type for a message object of type 'animal_adjust"
  "miro2_msg/animal_adjust")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<animal_adjust>)))
  "Returns md5sum for a message object of type '<animal_adjust>"
  "b26581aa1bf2879431400970feb511a2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'animal_adjust)))
  "Returns md5sum for a message object of type 'animal_adjust"
  "b26581aa1bf2879431400970feb511a2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<animal_adjust>)))
  "Returns full string definition for message of type '<animal_adjust>"
  (cl:format cl:nil "#	@section COPYRIGHT~%#	Copyright (C) 2019 Consequential Robotics Ltd~%#	~%#	@section AUTHOR~%#	Consequential Robotics http://consequentialrobotics.com~%#	~%#	@section LICENSE~%#	For a full copy of the license agreement, and a complete~%#	definition of \"The Software\", see LICENSE in the MDK root~%#	directory.~%#	~%#	Subject to the terms of this Agreement, Consequential~%#	Robotics grants to you a limited, non-exclusive, non-~%#	transferable license, without right to sub-license, to use~%#	\"The Software\" in accordance with this Agreement and any~%#	other written agreement with Consequential Robotics.~%#	Consequential Robotics does not transfer the title of \"The~%#	Software\" to you; the license granted to you is not a sale.~%#	This agreement is a binding legal agreement between~%#	Consequential Robotics and the purchasers or users of \"The~%#	Software\".~%#	~%#	THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY~%#	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE~%#	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR~%#	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS~%#	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR~%#	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR~%#	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE~%#	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.~%#	~%#~%#	Animal adjust provides a route for directly adjusting the~%#	animal state. See message \"animal_state\" for a description~%#	of the state itself; see message \"adjust\" for details of~%#	how adjustment can be performed.~%~%affect_adjust mood~%affect_adjust emotion~%sleep_adjust sleep~%~%~%================================================================================~%MSG: miro2_msg/affect_adjust~%#	@section COPYRIGHT~%#	Copyright (C) 2019 Consequential Robotics Ltd~%#	~%#	@section AUTHOR~%#	Consequential Robotics http://consequentialrobotics.com~%#	~%#	@section LICENSE~%#	For a full copy of the license agreement, and a complete~%#	definition of \"The Software\", see LICENSE in the MDK root~%#	directory.~%#	~%#	Subject to the terms of this Agreement, Consequential~%#	Robotics grants to you a limited, non-exclusive, non-~%#	transferable license, without right to sub-license, to use~%#	\"The Software\" in accordance with this Agreement and any~%#	other written agreement with Consequential Robotics.~%#	Consequential Robotics does not transfer the title of \"The~%#	Software\" to you; the license granted to you is not a sale.~%#	This agreement is a binding legal agreement between~%#	Consequential Robotics and the purchasers or users of \"The~%#	Software\".~%#	~%#	THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY~%#	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE~%#	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR~%#	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS~%#	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR~%#	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR~%#	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE~%#	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.~%#	~%~%adjust valence~%adjust arousal~%~%~%================================================================================~%MSG: miro2_msg/adjust~%#	@section COPYRIGHT~%#	Copyright (C) 2019 Consequential Robotics Ltd~%#	~%#	@section AUTHOR~%#	Consequential Robotics http://consequentialrobotics.com~%#	~%#	@section LICENSE~%#	For a full copy of the license agreement, and a complete~%#	definition of \"The Software\", see LICENSE in the MDK root~%#	directory.~%#	~%#	Subject to the terms of this Agreement, Consequential~%#	Robotics grants to you a limited, non-exclusive, non-~%#	transferable license, without right to sub-license, to use~%#	\"The Software\" in accordance with this Agreement and any~%#	other written agreement with Consequential Robotics.~%#	Consequential Robotics does not transfer the title of \"The~%#	Software\" to you; the license granted to you is not a sale.~%#	This agreement is a binding legal agreement between~%#	Consequential Robotics and the purchasers or users of \"The~%#	Software\".~%#	~%#	THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY~%#	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE~%#	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR~%#	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS~%#	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR~%#	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR~%#	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE~%#	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.~%#	~%#~%#	Adjust message provides a route for directly adjusting~%#	a state of the biomimetic model. There are two ways to~%#	specify an adjustment, selected independently for each~%#	adjustment channel.~%#~%#	1) Provide a target value in \"data\" and a \"gamma\" value~%#	between 0 and 1 to cause the state to approach the target:~%#~%#	(at 50Hz)~%#	state += gamma * (data - state)~%#~%#	2) Provide a delta value in \"data\" and set \"gamma\"~%#	to -1 to indicate this drive mode:~%#~%#	(at 50Hz)~%#	state += data~%#~%#	Understood values of gamma, therefore, are:~%#	   -1 : add \"data\" to state~%#	    0 : do nothing~%#	  0-1 : move state towards \"data\"~%#	    1 : instantly set state to \"data\"~%~%float32 data~%float32 gamma~%~%~%================================================================================~%MSG: miro2_msg/sleep_adjust~%#	@section COPYRIGHT~%#	Copyright (C) 2019 Consequential Robotics Ltd~%#	~%#	@section AUTHOR~%#	Consequential Robotics http://consequentialrobotics.com~%#	~%#	@section LICENSE~%#	For a full copy of the license agreement, and a complete~%#	definition of \"The Software\", see LICENSE in the MDK root~%#	directory.~%#	~%#	Subject to the terms of this Agreement, Consequential~%#	Robotics grants to you a limited, non-exclusive, non-~%#	transferable license, without right to sub-license, to use~%#	\"The Software\" in accordance with this Agreement and any~%#	other written agreement with Consequential Robotics.~%#	Consequential Robotics does not transfer the title of \"The~%#	Software\" to you; the license granted to you is not a sale.~%#	This agreement is a binding legal agreement between~%#	Consequential Robotics and the purchasers or users of \"The~%#	Software\".~%#	~%#	THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY~%#	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE~%#	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR~%#	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS~%#	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR~%#	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR~%#	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE~%#	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.~%#	~%~%adjust wakefulness~%adjust pressure~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'animal_adjust)))
  "Returns full string definition for message of type 'animal_adjust"
  (cl:format cl:nil "#	@section COPYRIGHT~%#	Copyright (C) 2019 Consequential Robotics Ltd~%#	~%#	@section AUTHOR~%#	Consequential Robotics http://consequentialrobotics.com~%#	~%#	@section LICENSE~%#	For a full copy of the license agreement, and a complete~%#	definition of \"The Software\", see LICENSE in the MDK root~%#	directory.~%#	~%#	Subject to the terms of this Agreement, Consequential~%#	Robotics grants to you a limited, non-exclusive, non-~%#	transferable license, without right to sub-license, to use~%#	\"The Software\" in accordance with this Agreement and any~%#	other written agreement with Consequential Robotics.~%#	Consequential Robotics does not transfer the title of \"The~%#	Software\" to you; the license granted to you is not a sale.~%#	This agreement is a binding legal agreement between~%#	Consequential Robotics and the purchasers or users of \"The~%#	Software\".~%#	~%#	THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY~%#	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE~%#	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR~%#	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS~%#	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR~%#	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR~%#	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE~%#	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.~%#	~%#~%#	Animal adjust provides a route for directly adjusting the~%#	animal state. See message \"animal_state\" for a description~%#	of the state itself; see message \"adjust\" for details of~%#	how adjustment can be performed.~%~%affect_adjust mood~%affect_adjust emotion~%sleep_adjust sleep~%~%~%================================================================================~%MSG: miro2_msg/affect_adjust~%#	@section COPYRIGHT~%#	Copyright (C) 2019 Consequential Robotics Ltd~%#	~%#	@section AUTHOR~%#	Consequential Robotics http://consequentialrobotics.com~%#	~%#	@section LICENSE~%#	For a full copy of the license agreement, and a complete~%#	definition of \"The Software\", see LICENSE in the MDK root~%#	directory.~%#	~%#	Subject to the terms of this Agreement, Consequential~%#	Robotics grants to you a limited, non-exclusive, non-~%#	transferable license, without right to sub-license, to use~%#	\"The Software\" in accordance with this Agreement and any~%#	other written agreement with Consequential Robotics.~%#	Consequential Robotics does not transfer the title of \"The~%#	Software\" to you; the license granted to you is not a sale.~%#	This agreement is a binding legal agreement between~%#	Consequential Robotics and the purchasers or users of \"The~%#	Software\".~%#	~%#	THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY~%#	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE~%#	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR~%#	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS~%#	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR~%#	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR~%#	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE~%#	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.~%#	~%~%adjust valence~%adjust arousal~%~%~%================================================================================~%MSG: miro2_msg/adjust~%#	@section COPYRIGHT~%#	Copyright (C) 2019 Consequential Robotics Ltd~%#	~%#	@section AUTHOR~%#	Consequential Robotics http://consequentialrobotics.com~%#	~%#	@section LICENSE~%#	For a full copy of the license agreement, and a complete~%#	definition of \"The Software\", see LICENSE in the MDK root~%#	directory.~%#	~%#	Subject to the terms of this Agreement, Consequential~%#	Robotics grants to you a limited, non-exclusive, non-~%#	transferable license, without right to sub-license, to use~%#	\"The Software\" in accordance with this Agreement and any~%#	other written agreement with Consequential Robotics.~%#	Consequential Robotics does not transfer the title of \"The~%#	Software\" to you; the license granted to you is not a sale.~%#	This agreement is a binding legal agreement between~%#	Consequential Robotics and the purchasers or users of \"The~%#	Software\".~%#	~%#	THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY~%#	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE~%#	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR~%#	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS~%#	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR~%#	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR~%#	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE~%#	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.~%#	~%#~%#	Adjust message provides a route for directly adjusting~%#	a state of the biomimetic model. There are two ways to~%#	specify an adjustment, selected independently for each~%#	adjustment channel.~%#~%#	1) Provide a target value in \"data\" and a \"gamma\" value~%#	between 0 and 1 to cause the state to approach the target:~%#~%#	(at 50Hz)~%#	state += gamma * (data - state)~%#~%#	2) Provide a delta value in \"data\" and set \"gamma\"~%#	to -1 to indicate this drive mode:~%#~%#	(at 50Hz)~%#	state += data~%#~%#	Understood values of gamma, therefore, are:~%#	   -1 : add \"data\" to state~%#	    0 : do nothing~%#	  0-1 : move state towards \"data\"~%#	    1 : instantly set state to \"data\"~%~%float32 data~%float32 gamma~%~%~%================================================================================~%MSG: miro2_msg/sleep_adjust~%#	@section COPYRIGHT~%#	Copyright (C) 2019 Consequential Robotics Ltd~%#	~%#	@section AUTHOR~%#	Consequential Robotics http://consequentialrobotics.com~%#	~%#	@section LICENSE~%#	For a full copy of the license agreement, and a complete~%#	definition of \"The Software\", see LICENSE in the MDK root~%#	directory.~%#	~%#	Subject to the terms of this Agreement, Consequential~%#	Robotics grants to you a limited, non-exclusive, non-~%#	transferable license, without right to sub-license, to use~%#	\"The Software\" in accordance with this Agreement and any~%#	other written agreement with Consequential Robotics.~%#	Consequential Robotics does not transfer the title of \"The~%#	Software\" to you; the license granted to you is not a sale.~%#	This agreement is a binding legal agreement between~%#	Consequential Robotics and the purchasers or users of \"The~%#	Software\".~%#	~%#	THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY~%#	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE~%#	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR~%#	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS~%#	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR~%#	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR~%#	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE~%#	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.~%#	~%~%adjust wakefulness~%adjust pressure~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <animal_adjust>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'mood))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'emotion))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'sleep))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <animal_adjust>))
  "Converts a ROS message object to a list"
  (cl:list 'animal_adjust
    (cl:cons ':mood (mood msg))
    (cl:cons ':emotion (emotion msg))
    (cl:cons ':sleep (sleep msg))
))
