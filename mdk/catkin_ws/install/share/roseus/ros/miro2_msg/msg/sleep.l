;; Auto-generated. Do not edit!


(when (boundp 'miro2_msg::sleep)
  (if (not (find-package "MIRO2_MSG"))
    (make-package "MIRO2_MSG"))
  (shadow 'sleep (find-package "MIRO2_MSG")))
(unless (find-package "MIRO2_MSG::SLEEP")
  (make-package "MIRO2_MSG::SLEEP"))

(in-package "ROS")
;;//! \htmlinclude sleep.msg.html


(defclass miro2_msg::sleep
  :super ros::object
  :slots (_wakefulness _pressure ))

(defmethod miro2_msg::sleep
  (:init
   (&key
    ((:wakefulness __wakefulness) 0.0)
    ((:pressure __pressure) 0.0)
    )
   (send-super :init)
   (setq _wakefulness (float __wakefulness))
   (setq _pressure (float __pressure))
   self)
  (:wakefulness
   (&optional __wakefulness)
   (if __wakefulness (setq _wakefulness __wakefulness)) _wakefulness)
  (:pressure
   (&optional __pressure)
   (if __pressure (setq _pressure __pressure)) _pressure)
  (:serialization-length
   ()
   (+
    ;; float32 _wakefulness
    4
    ;; float32 _pressure
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _wakefulness
       (sys::poke _wakefulness (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pressure
       (sys::poke _pressure (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _wakefulness
     (setq _wakefulness (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pressure
     (setq _pressure (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get miro2_msg::sleep :md5sum-) "9ae301b8349f95e1749450e5431eef09")
(setf (get miro2_msg::sleep :datatype-) "miro2_msg/sleep")
(setf (get miro2_msg::sleep :definition-)
      "#	@section COPYRIGHT
#	Copyright (C) 2019 Consequential Robotics Ltd
#	
#	@section AUTHOR
#	Consequential Robotics http://consequentialrobotics.com
#	
#	@section LICENSE
#	For a full copy of the license agreement, see LICENSE in the
#	MDK root directory.
#	
#	Subject to the terms of this Agreement, Consequential
#	Robotics grants to you a limited, non-exclusive, non-
#	transferable license, without right to sub-license, to use
#	MIRO Developer Kit in accordance with this Agreement and any
#	other written agreement with Consequential Robotics.
#	Consequential Robotics does not transfer the title of MIRO
#	Developer Kit to you; the license granted to you is not a
#	sale. This agreement is a binding legal agreement between
#	Consequential Robotics and the purchasers or users of MIRO
#	Developer Kit.
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
#	The \"sleep\" state is two-dimensional, encoding \"wakefulness\"
#	(0.0 to 1.0, what it sounds like) and \"pressure\" (0.0 to 1.0,
#	tendency to move towards reduced wakefulness). The two states
#	evolve together to implement a relaxation oscillator.

float32 wakefulness
float32 pressure


")



(provide :miro2_msg/sleep "9ae301b8349f95e1749450e5431eef09")


