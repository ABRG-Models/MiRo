;; Auto-generated. Do not edit!


(when (boundp 'miro2_msg::animal_adjust)
  (if (not (find-package "MIRO2_MSG"))
    (make-package "MIRO2_MSG"))
  (shadow 'animal_adjust (find-package "MIRO2_MSG")))
(unless (find-package "MIRO2_MSG::ANIMAL_ADJUST")
  (make-package "MIRO2_MSG::ANIMAL_ADJUST"))

(in-package "ROS")
;;//! \htmlinclude animal_adjust.msg.html


(defclass miro2_msg::animal_adjust
  :super ros::object
  :slots (_mood _emotion _sleep ))

(defmethod miro2_msg::animal_adjust
  (:init
   (&key
    ((:mood __mood) (instance miro2_msg::affect_adjust :init))
    ((:emotion __emotion) (instance miro2_msg::affect_adjust :init))
    ((:sleep __sleep) (instance miro2_msg::sleep_adjust :init))
    )
   (send-super :init)
   (setq _mood __mood)
   (setq _emotion __emotion)
   (setq _sleep __sleep)
   self)
  (:mood
   (&rest __mood)
   (if (keywordp (car __mood))
       (send* _mood __mood)
     (progn
       (if __mood (setq _mood (car __mood)))
       _mood)))
  (:emotion
   (&rest __emotion)
   (if (keywordp (car __emotion))
       (send* _emotion __emotion)
     (progn
       (if __emotion (setq _emotion (car __emotion)))
       _emotion)))
  (:sleep
   (&rest __sleep)
   (if (keywordp (car __sleep))
       (send* _sleep __sleep)
     (progn
       (if __sleep (setq _sleep (car __sleep)))
       _sleep)))
  (:serialization-length
   ()
   (+
    ;; miro2_msg/affect_adjust _mood
    (send _mood :serialization-length)
    ;; miro2_msg/affect_adjust _emotion
    (send _emotion :serialization-length)
    ;; miro2_msg/sleep_adjust _sleep
    (send _sleep :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; miro2_msg/affect_adjust _mood
       (send _mood :serialize s)
     ;; miro2_msg/affect_adjust _emotion
       (send _emotion :serialize s)
     ;; miro2_msg/sleep_adjust _sleep
       (send _sleep :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; miro2_msg/affect_adjust _mood
     (send _mood :deserialize buf ptr-) (incf ptr- (send _mood :serialization-length))
   ;; miro2_msg/affect_adjust _emotion
     (send _emotion :deserialize buf ptr-) (incf ptr- (send _emotion :serialization-length))
   ;; miro2_msg/sleep_adjust _sleep
     (send _sleep :deserialize buf ptr-) (incf ptr- (send _sleep :serialization-length))
   ;;
   self)
  )

(setf (get miro2_msg::animal_adjust :md5sum-) "b26581aa1bf2879431400970feb511a2")
(setf (get miro2_msg::animal_adjust :datatype-) "miro2_msg/animal_adjust")
(setf (get miro2_msg::animal_adjust :definition-)
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
#	Animal adjust provides a route for directly adjusting the
#	animal state. See message \"animal_state\" for a description
#	of the state itself; see message \"adjust\" for details of
#	how adjustment can be performed.

affect_adjust mood
affect_adjust emotion
sleep_adjust sleep


================================================================================
MSG: miro2_msg/affect_adjust
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


================================================================================
MSG: miro2_msg/sleep_adjust
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

adjust wakefulness
adjust pressure


")



(provide :miro2_msg/animal_adjust "b26581aa1bf2879431400970feb511a2")


