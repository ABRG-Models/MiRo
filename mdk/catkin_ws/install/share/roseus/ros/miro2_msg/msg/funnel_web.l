;; Auto-generated. Do not edit!


(when (boundp 'miro2_msg::funnel_web)
  (if (not (find-package "MIRO2_MSG"))
    (make-package "MIRO2_MSG"))
  (shadow 'funnel_web (find-package "MIRO2_MSG")))
(unless (find-package "MIRO2_MSG::FUNNEL_WEB")
  (make-package "MIRO2_MSG::FUNNEL_WEB"))

(in-package "ROS")
;;//! \htmlinclude funnel_web.msg.html


(defclass miro2_msg::funnel_web
  :super ros::object
  :slots (_cliff _light _sonar _touch _illum _audio_level _tone ))

(defmethod miro2_msg::funnel_web
  (:init
   (&key
    ((:cliff __cliff) (make-array 2 :initial-element 0.0 :element-type :float))
    ((:light __light) (make-array 4 :initial-element 0.0 :element-type :float))
    ((:sonar __sonar) 0.0)
    ((:touch __touch) (make-array 2 :initial-element 0 :element-type :integer))
    ((:illum __illum) (make-array 6 :initial-element 0 :element-type :integer))
    ((:audio_level __audio_level) 0.0)
    ((:tone __tone) (make-array 3 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _cliff __cliff)
   (setq _light __light)
   (setq _sonar (float __sonar))
   (setq _touch __touch)
   (setq _illum __illum)
   (setq _audio_level (float __audio_level))
   (setq _tone __tone)
   self)
  (:cliff
   (&optional __cliff)
   (if __cliff (setq _cliff __cliff)) _cliff)
  (:light
   (&optional __light)
   (if __light (setq _light __light)) _light)
  (:sonar
   (&optional __sonar)
   (if __sonar (setq _sonar __sonar)) _sonar)
  (:touch
   (&optional __touch)
   (if __touch (setq _touch __touch)) _touch)
  (:illum
   (&optional __illum)
   (if __illum (setq _illum __illum)) _illum)
  (:audio_level
   (&optional __audio_level)
   (if __audio_level (setq _audio_level __audio_level)) _audio_level)
  (:tone
   (&optional __tone)
   (if __tone (setq _tone __tone)) _tone)
  (:serialization-length
   ()
   (+
    ;; float32[2] _cliff
    (* 4    2)
    ;; float32[4] _light
    (* 4    4)
    ;; float32 _sonar
    4
    ;; uint16[2] _touch
    (* 2    2)
    ;; uint32[6] _illum
    (* 4    6)
    ;; float32 _audio_level
    4
    ;; uint16[3] _tone
    (* 2    3)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[2] _cliff
     (dotimes (i 2)
       (sys::poke (elt _cliff i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[4] _light
     (dotimes (i 4)
       (sys::poke (elt _light i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32 _sonar
       (sys::poke _sonar (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint16[2] _touch
     (dotimes (i 2)
       (write-word (elt _touch i) s)
       )
     ;; uint32[6] _illum
     (dotimes (i 6)
       (write-long (elt _illum i) s)
       )
     ;; float32 _audio_level
       (sys::poke _audio_level (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint16[3] _tone
     (dotimes (i 3)
       (write-word (elt _tone i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[2] _cliff
   (dotimes (i (length _cliff))
     (setf (elt _cliff i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[4] _light
   (dotimes (i (length _light))
     (setf (elt _light i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32 _sonar
     (setq _sonar (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint16[2] _touch
   (dotimes (i (length _touch))
     (setf (elt _touch i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;; uint32[6] _illum
   (dotimes (i (length _illum))
     (setf (elt _illum i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;; float32 _audio_level
     (setq _audio_level (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint16[3] _tone
   (dotimes (i (length _tone))
     (setf (elt _tone i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;;
   self)
  )

(setf (get miro2_msg::funnel_web :md5sum-) "d76c86fb9327536412ade3373c6dd88e")
(setf (get miro2_msg::funnel_web :datatype-) "miro2_msg/funnel_web")
(setf (get miro2_msg::funnel_web :definition-)
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
#	This message packages several messages for simple
#	delivery to the web client.



float32[2] cliff
float32[4] light
float32 sonar
uint16[2] touch
uint32[6] illum
float32 audio_level
uint16[3] tone





")



(provide :miro2_msg/funnel_web "d76c86fb9327536412ade3373c6dd88e")


