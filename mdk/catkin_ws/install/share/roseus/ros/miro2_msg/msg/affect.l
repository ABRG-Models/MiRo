;; Auto-generated. Do not edit!


(when (boundp 'miro2_msg::affect)
  (if (not (find-package "MIRO2_MSG"))
    (make-package "MIRO2_MSG"))
  (shadow 'affect (find-package "MIRO2_MSG")))
(unless (find-package "MIRO2_MSG::AFFECT")
  (make-package "MIRO2_MSG::AFFECT"))

(in-package "ROS")
;;//! \htmlinclude affect.msg.html


(defclass miro2_msg::affect
  :super ros::object
  :slots (_valence _arousal ))

(defmethod miro2_msg::affect
  (:init
   (&key
    ((:valence __valence) 0.0)
    ((:arousal __arousal) 0.0)
    )
   (send-super :init)
   (setq _valence (float __valence))
   (setq _arousal (float __arousal))
   self)
  (:valence
   (&optional __valence)
   (if __valence (setq _valence __valence)) _valence)
  (:arousal
   (&optional __arousal)
   (if __arousal (setq _arousal __arousal)) _arousal)
  (:serialization-length
   ()
   (+
    ;; float32 _valence
    4
    ;; float32 _arousal
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _valence
       (sys::poke _valence (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _arousal
       (sys::poke _arousal (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _valence
     (setq _valence (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _arousal
     (setq _arousal (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get miro2_msg::affect :md5sum-) "b9db7d9709bc98cc560b83ce0bc4f004")
(setf (get miro2_msg::affect :datatype-) "miro2_msg/affect")
(setf (get miro2_msg::affect :definition-)
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
#	The \"affect\" state is two-dimensional, encoding valence
#	(-ve = sad, +ve = happy) and arousal (-ve = relaxed, +ve = alert).
#	The states are usually driven by signals entering the robot's
#	sensory systems, but can also be driven directly by other systems.

float32 valence
float32 arousal


")



(provide :miro2_msg/affect "b9db7d9709bc98cc560b83ce0bc4f004")


