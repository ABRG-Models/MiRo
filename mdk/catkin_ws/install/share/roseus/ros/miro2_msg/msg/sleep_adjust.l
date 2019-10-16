;; Auto-generated. Do not edit!


(when (boundp 'miro2_msg::sleep_adjust)
  (if (not (find-package "MIRO2_MSG"))
    (make-package "MIRO2_MSG"))
  (shadow 'sleep_adjust (find-package "MIRO2_MSG")))
(unless (find-package "MIRO2_MSG::SLEEP_ADJUST")
  (make-package "MIRO2_MSG::SLEEP_ADJUST"))

(in-package "ROS")
;;//! \htmlinclude sleep_adjust.msg.html


(defclass miro2_msg::sleep_adjust
  :super ros::object
  :slots (_wakefulness _pressure ))

(defmethod miro2_msg::sleep_adjust
  (:init
   (&key
    ((:wakefulness __wakefulness) (instance miro2_msg::adjust :init))
    ((:pressure __pressure) (instance miro2_msg::adjust :init))
    )
   (send-super :init)
   (setq _wakefulness __wakefulness)
   (setq _pressure __pressure)
   self)
  (:wakefulness
   (&rest __wakefulness)
   (if (keywordp (car __wakefulness))
       (send* _wakefulness __wakefulness)
     (progn
       (if __wakefulness (setq _wakefulness (car __wakefulness)))
       _wakefulness)))
  (:pressure
   (&rest __pressure)
   (if (keywordp (car __pressure))
       (send* _pressure __pressure)
     (progn
       (if __pressure (setq _pressure (car __pressure)))
       _pressure)))
  (:serialization-length
   ()
   (+
    ;; miro2_msg/adjust _wakefulness
    (send _wakefulness :serialization-length)
    ;; miro2_msg/adjust _pressure
    (send _pressure :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; miro2_msg/adjust _wakefulness
       (send _wakefulness :serialize s)
     ;; miro2_msg/adjust _pressure
       (send _pressure :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; miro2_msg/adjust _wakefulness
     (send _wakefulness :deserialize buf ptr-) (incf ptr- (send _wakefulness :serialization-length))
   ;; miro2_msg/adjust _pressure
     (send _pressure :deserialize buf ptr-) (incf ptr- (send _pressure :serialization-length))
   ;;
   self)
  )

(setf (get miro2_msg::sleep_adjust :md5sum-) "d50186fa3ccd96b438a40f38889ec949")
(setf (get miro2_msg::sleep_adjust :datatype-) "miro2_msg/sleep_adjust")
(setf (get miro2_msg::sleep_adjust :definition-)
      "#	@section COPYRIGHT
#	Copyright (C) 2019 Consequential Robotics Ltd
#	
#	@section AUTHOR
#	Consequential Robotics http://consequentialrobotics.com
#	
#	@section LICENSE
#	For a full copy of the license agreement, and a complete
#	definition of \"The Software\", see LICENSE in the MDK root
#	directory.
#	
#	Subject to the terms of this Agreement, Consequential
#	Robotics grants to you a limited, non-exclusive, non-
#	transferable license, without right to sub-license, to use
#	\"The Software\" in accordance with this Agreement and any
#	other written agreement with Consequential Robotics.
#	Consequential Robotics does not transfer the title of \"The
#	Software\" to you; the license granted to you is not a sale.
#	This agreement is a binding legal agreement between
#	Consequential Robotics and the purchasers or users of \"The
#	Software\".
#	
#	THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY
#	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
#	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
#	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
#	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
#	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
#	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
#	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#	

adjust wakefulness
adjust pressure


================================================================================
MSG: miro2_msg/adjust
#	@section COPYRIGHT
#	Copyright (C) 2019 Consequential Robotics Ltd
#	
#	@section AUTHOR
#	Consequential Robotics http://consequentialrobotics.com
#	
#	@section LICENSE
#	For a full copy of the license agreement, and a complete
#	definition of \"The Software\", see LICENSE in the MDK root
#	directory.
#	
#	Subject to the terms of this Agreement, Consequential
#	Robotics grants to you a limited, non-exclusive, non-
#	transferable license, without right to sub-license, to use
#	\"The Software\" in accordance with this Agreement and any
#	other written agreement with Consequential Robotics.
#	Consequential Robotics does not transfer the title of \"The
#	Software\" to you; the license granted to you is not a sale.
#	This agreement is a binding legal agreement between
#	Consequential Robotics and the purchasers or users of \"The
#	Software\".
#	
#	THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY
#	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
#	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
#	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
#	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
#	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
#	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
#	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#	
#
#	Adjust message provides a route for directly adjusting
#	a state of the biomimetic model. There are two ways to
#	specify an adjustment, selected independently for each
#	adjustment channel.
#
#	1) Provide a target value in \"data\" and a \"gamma\" value
#	between 0 and 1 to cause the state to approach the target:
#
#	(at 50Hz)
#	state += gamma * (data - state)
#
#	2) Provide a delta value in \"data\" and set \"gamma\"
#	to -1 to indicate this drive mode:
#
#	(at 50Hz)
#	state += data
#
#	Understood values of gamma, therefore, are:
#	   -1 : add \"data\" to state
#	    0 : do nothing
#	  0-1 : move state towards \"data\"
#	    1 : instantly set state to \"data\"

float32 data
float32 gamma


")



(provide :miro2_msg/sleep_adjust "d50186fa3ccd96b438a40f38889ec949")


