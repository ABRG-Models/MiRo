;; Auto-generated. Do not edit!


(when (boundp 'miro2_msg::voice_state)
  (if (not (find-package "MIRO2_MSG"))
    (make-package "MIRO2_MSG"))
  (shadow 'voice_state (find-package "MIRO2_MSG")))
(unless (find-package "MIRO2_MSG::VOICE_STATE")
  (make-package "MIRO2_MSG::VOICE_STATE"))

(in-package "ROS")
;;//! \htmlinclude voice_state.msg.html


(defclass miro2_msg::voice_state
  :super ros::object
  :slots (_breathing_phase _vocalising ))

(defmethod miro2_msg::voice_state
  (:init
   (&key
    ((:breathing_phase __breathing_phase) 0.0)
    ((:vocalising __vocalising) nil)
    )
   (send-super :init)
   (setq _breathing_phase (float __breathing_phase))
   (setq _vocalising __vocalising)
   self)
  (:breathing_phase
   (&optional __breathing_phase)
   (if __breathing_phase (setq _breathing_phase __breathing_phase)) _breathing_phase)
  (:vocalising
   (&optional __vocalising)
   (if __vocalising (setq _vocalising __vocalising)) _vocalising)
  (:serialization-length
   ()
   (+
    ;; float32 _breathing_phase
    4
    ;; bool _vocalising
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _breathing_phase
       (sys::poke _breathing_phase (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _vocalising
       (if _vocalising (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _breathing_phase
     (setq _breathing_phase (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _vocalising
     (setq _vocalising (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get miro2_msg::voice_state :md5sum-) "e821a66f37dcfd027ec1d69a1734ae31")
(setf (get miro2_msg::voice_state :datatype-) "miro2_msg/voice_state")
(setf (get miro2_msg::voice_state :definition-)
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

float32 breathing_phase
bool vocalising


")



(provide :miro2_msg/voice_state "e821a66f37dcfd027ec1d69a1734ae31")


