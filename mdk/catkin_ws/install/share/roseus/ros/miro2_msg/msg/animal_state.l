;; Auto-generated. Do not edit!


(when (boundp 'miro2_msg::animal_state)
  (if (not (find-package "MIRO2_MSG"))
    (make-package "MIRO2_MSG"))
  (shadow 'animal_state (find-package "MIRO2_MSG")))
(unless (find-package "MIRO2_MSG::ANIMAL_STATE")
  (make-package "MIRO2_MSG::ANIMAL_STATE"))

(in-package "ROS")
;;//! \htmlinclude animal_state.msg.html


(defclass miro2_msg::animal_state
  :super ros::object
  :slots (_flags _emotion _mood _sleep _time_of_day _sound_level ))

(defmethod miro2_msg::animal_state
  (:init
   (&key
    ((:flags __flags) 0)
    ((:emotion __emotion) (instance miro2_msg::affect :init))
    ((:mood __mood) (instance miro2_msg::affect :init))
    ((:sleep __sleep) (instance miro2_msg::sleep :init))
    ((:time_of_day __time_of_day) 0.0)
    ((:sound_level __sound_level) 0.0)
    )
   (send-super :init)
   (setq _flags (round __flags))
   (setq _emotion __emotion)
   (setq _mood __mood)
   (setq _sleep __sleep)
   (setq _time_of_day (float __time_of_day))
   (setq _sound_level (float __sound_level))
   self)
  (:flags
   (&optional __flags)
   (if __flags (setq _flags __flags)) _flags)
  (:emotion
   (&rest __emotion)
   (if (keywordp (car __emotion))
       (send* _emotion __emotion)
     (progn
       (if __emotion (setq _emotion (car __emotion)))
       _emotion)))
  (:mood
   (&rest __mood)
   (if (keywordp (car __mood))
       (send* _mood __mood)
     (progn
       (if __mood (setq _mood (car __mood)))
       _mood)))
  (:sleep
   (&rest __sleep)
   (if (keywordp (car __sleep))
       (send* _sleep __sleep)
     (progn
       (if __sleep (setq _sleep (car __sleep)))
       _sleep)))
  (:time_of_day
   (&optional __time_of_day)
   (if __time_of_day (setq _time_of_day __time_of_day)) _time_of_day)
  (:sound_level
   (&optional __sound_level)
   (if __sound_level (setq _sound_level __sound_level)) _sound_level)
  (:serialization-length
   ()
   (+
    ;; uint32 _flags
    4
    ;; miro2_msg/affect _emotion
    (send _emotion :serialization-length)
    ;; miro2_msg/affect _mood
    (send _mood :serialization-length)
    ;; miro2_msg/sleep _sleep
    (send _sleep :serialization-length)
    ;; float32 _time_of_day
    4
    ;; float32 _sound_level
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _flags
       (write-long _flags s)
     ;; miro2_msg/affect _emotion
       (send _emotion :serialize s)
     ;; miro2_msg/affect _mood
       (send _mood :serialize s)
     ;; miro2_msg/sleep _sleep
       (send _sleep :serialize s)
     ;; float32 _time_of_day
       (sys::poke _time_of_day (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _sound_level
       (sys::poke _sound_level (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _flags
     (setq _flags (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; miro2_msg/affect _emotion
     (send _emotion :deserialize buf ptr-) (incf ptr- (send _emotion :serialization-length))
   ;; miro2_msg/affect _mood
     (send _mood :deserialize buf ptr-) (incf ptr- (send _mood :serialization-length))
   ;; miro2_msg/sleep _sleep
     (send _sleep :deserialize buf ptr-) (incf ptr- (send _sleep :serialization-length))
   ;; float32 _time_of_day
     (setq _time_of_day (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _sound_level
     (setq _sound_level (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get miro2_msg::animal_state :md5sum-) "02b89a84b06f59e91819662e7c3d6b0e")
(setf (get miro2_msg::animal_state :datatype-) "miro2_msg/animal_state")
(setf (get miro2_msg::animal_state :definition-)
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
# the \"animal_state\" is the state of the animal aspects of the model
# which includes \"affect\" values for emotion (quickly changing) and
# mood (slowly changing) and a \"sleep\" value (also slowly changing),
# and the animal's estimate of time. this state may also include, in
# future, physical states such as temperature.

#	DOCLINK ANIMAL STATE FLAGS
#
#	Some flags are included here because parts of the implementation
#	are in separate nodes that read this topic in order to determine
#	how they should behave, and their behaviour is affected by flags.
#
#	The values of these flags are defined in miro2.constants.
uint32 flags

# affective states
affect emotion
affect mood

# sleep state
sleep sleep

# normalised time of day (0.0 -> 1.0)
float32 time_of_day

# normalised ambient sound level (0.0 -> 1.0)
float32 sound_level

================================================================================
MSG: miro2_msg/affect
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
#	The \"affect\" state is two-dimensional, encoding valence
#	(-ve = sad, +ve = happy) and arousal (-ve = relaxed, +ve = alert).
#	The states are usually driven by signals entering the robot's
#	sensory systems, but can also be driven directly by other systems.

float32 valence
float32 arousal


================================================================================
MSG: miro2_msg/sleep
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
#	The \"sleep\" state is two-dimensional, encoding \"wakefulness\"
#	(0.0 to 1.0, what it sounds like) and \"pressure\" (0.0 to 1.0,
#	tendency to move towards reduced wakefulness). The two states
#	evolve together to implement a relaxation oscillator.

float32 wakefulness
float32 pressure


")



(provide :miro2_msg/animal_state "02b89a84b06f59e91819662e7c3d6b0e")


