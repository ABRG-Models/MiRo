;; Auto-generated. Do not edit!


(when (boundp 'miro2_msg::adjust)
  (if (not (find-package "MIRO2_MSG"))
    (make-package "MIRO2_MSG"))
  (shadow 'adjust (find-package "MIRO2_MSG")))
(unless (find-package "MIRO2_MSG::ADJUST")
  (make-package "MIRO2_MSG::ADJUST"))

(in-package "ROS")
;;//! \htmlinclude adjust.msg.html


(defclass miro2_msg::adjust
  :super ros::object
  :slots (_data _gamma ))

(defmethod miro2_msg::adjust
  (:init
   (&key
    ((:data __data) 0.0)
    ((:gamma __gamma) 0.0)
    )
   (send-super :init)
   (setq _data (float __data))
   (setq _gamma (float __gamma))
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:gamma
   (&optional __gamma)
   (if __gamma (setq _gamma __gamma)) _gamma)
  (:serialization-length
   ()
   (+
    ;; float32 _data
    4
    ;; float32 _gamma
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _data
       (sys::poke _data (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _gamma
       (sys::poke _gamma (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _data
     (setq _data (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _gamma
     (setq _gamma (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get miro2_msg::adjust :md5sum-) "f57be85f314b6a6c4ab78af0ca827a06")
(setf (get miro2_msg::adjust :datatype-) "miro2_msg/adjust")
(setf (get miro2_msg::adjust :definition-)
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



(provide :miro2_msg/adjust "f57be85f314b6a6c4ab78af0ca827a06")


