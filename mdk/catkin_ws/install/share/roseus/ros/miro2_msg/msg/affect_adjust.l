;; Auto-generated. Do not edit!


(when (boundp 'miro2_msg::affect_adjust)
  (if (not (find-package "MIRO2_MSG"))
    (make-package "MIRO2_MSG"))
  (shadow 'affect_adjust (find-package "MIRO2_MSG")))
(unless (find-package "MIRO2_MSG::AFFECT_ADJUST")
  (make-package "MIRO2_MSG::AFFECT_ADJUST"))

(in-package "ROS")
;;//! \htmlinclude affect_adjust.msg.html


(defclass miro2_msg::affect_adjust
  :super ros::object
  :slots (_valence _arousal ))

(defmethod miro2_msg::affect_adjust
  (:init
   (&key
    ((:valence __valence) (instance miro2_msg::adjust :init))
    ((:arousal __arousal) (instance miro2_msg::adjust :init))
    )
   (send-super :init)
   (setq _valence __valence)
   (setq _arousal __arousal)
   self)
  (:valence
   (&rest __valence)
   (if (keywordp (car __valence))
       (send* _valence __valence)
     (progn
       (if __valence (setq _valence (car __valence)))
       _valence)))
  (:arousal
   (&rest __arousal)
   (if (keywordp (car __arousal))
       (send* _arousal __arousal)
     (progn
       (if __arousal (setq _arousal (car __arousal)))
       _arousal)))
  (:serialization-length
   ()
   (+
    ;; miro2_msg/adjust _valence
    (send _valence :serialization-length)
    ;; miro2_msg/adjust _arousal
    (send _arousal :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; miro2_msg/adjust _valence
       (send _valence :serialize s)
     ;; miro2_msg/adjust _arousal
       (send _arousal :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; miro2_msg/adjust _valence
     (send _valence :deserialize buf ptr-) (incf ptr- (send _valence :serialization-length))
   ;; miro2_msg/adjust _arousal
     (send _arousal :deserialize buf ptr-) (incf ptr- (send _arousal :serialization-length))
   ;;
   self)
  )

(setf (get miro2_msg::affect_adjust :md5sum-) "80f7164a0f33b02beb56e574f19360cc")
(setf (get miro2_msg::affect_adjust :datatype-) "miro2_msg/affect_adjust")
(setf (get miro2_msg::affect_adjust :definition-)
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

adjust valence
adjust arousal


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



(provide :miro2_msg/affect_adjust "80f7164a0f33b02beb56e574f19360cc")


