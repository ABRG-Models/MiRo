;; Auto-generated. Do not edit!


(when (boundp 'miro2_msg::priority_peak)
  (if (not (find-package "MIRO2_MSG"))
    (make-package "MIRO2_MSG"))
  (shadow 'priority_peak (find-package "MIRO2_MSG")))
(unless (find-package "MIRO2_MSG::PRIORITY_PEAK")
  (make-package "MIRO2_MSG::PRIORITY_PEAK"))

(in-package "ROS")
;;//! \htmlinclude priority_peak.msg.html


(defclass miro2_msg::priority_peak
  :super ros::object
  :slots (_stream_index _loc_d _height _size _azim _elev _size_norm _volume _range _actioned ))

(defmethod miro2_msg::priority_peak
  (:init
   (&key
    ((:stream_index __stream_index) 0)
    ((:loc_d __loc_d) (make-array 2 :initial-element 0.0 :element-type :float))
    ((:height __height) 0.0)
    ((:size __size) 0.0)
    ((:azim __azim) 0.0)
    ((:elev __elev) 0.0)
    ((:size_norm __size_norm) 0.0)
    ((:volume __volume) 0.0)
    ((:range __range) 0.0)
    ((:actioned __actioned) 0)
    )
   (send-super :init)
   (setq _stream_index (round __stream_index))
   (setq _loc_d __loc_d)
   (setq _height (float __height))
   (setq _size (float __size))
   (setq _azim (float __azim))
   (setq _elev (float __elev))
   (setq _size_norm (float __size_norm))
   (setq _volume (float __volume))
   (setq _range (float __range))
   (setq _actioned (round __actioned))
   self)
  (:stream_index
   (&optional __stream_index)
   (if __stream_index (setq _stream_index __stream_index)) _stream_index)
  (:loc_d
   (&optional __loc_d)
   (if __loc_d (setq _loc_d __loc_d)) _loc_d)
  (:height
   (&optional __height)
   (if __height (setq _height __height)) _height)
  (:size
   (&optional __size)
   (if __size (setq _size __size)) _size)
  (:azim
   (&optional __azim)
   (if __azim (setq _azim __azim)) _azim)
  (:elev
   (&optional __elev)
   (if __elev (setq _elev __elev)) _elev)
  (:size_norm
   (&optional __size_norm)
   (if __size_norm (setq _size_norm __size_norm)) _size_norm)
  (:volume
   (&optional __volume)
   (if __volume (setq _volume __volume)) _volume)
  (:range
   (&optional __range)
   (if __range (setq _range __range)) _range)
  (:actioned
   (&optional __actioned)
   (if __actioned (setq _actioned __actioned)) _actioned)
  (:serialization-length
   ()
   (+
    ;; int32 _stream_index
    4
    ;; float32[2] _loc_d
    (* 4    2)
    ;; float32 _height
    4
    ;; float32 _size
    4
    ;; float32 _azim
    4
    ;; float32 _elev
    4
    ;; float32 _size_norm
    4
    ;; float32 _volume
    4
    ;; float32 _range
    4
    ;; int32 _actioned
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _stream_index
       (write-long _stream_index s)
     ;; float32[2] _loc_d
     (dotimes (i 2)
       (sys::poke (elt _loc_d i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32 _height
       (sys::poke _height (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _size
       (sys::poke _size (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _azim
       (sys::poke _azim (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _elev
       (sys::poke _elev (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _size_norm
       (sys::poke _size_norm (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _volume
       (sys::poke _volume (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _range
       (sys::poke _range (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _actioned
       (write-long _actioned s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _stream_index
     (setq _stream_index (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32[2] _loc_d
   (dotimes (i (length _loc_d))
     (setf (elt _loc_d i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32 _height
     (setq _height (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _size
     (setq _size (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _azim
     (setq _azim (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _elev
     (setq _elev (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _size_norm
     (setq _size_norm (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _volume
     (setq _volume (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _range
     (setq _range (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _actioned
     (setq _actioned (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get miro2_msg::priority_peak :md5sum-) "d2924675ffe77da16f66ce7eb1cbeb6d")
(setf (get miro2_msg::priority_peak :datatype-) "miro2_msg/priority_peak")
(setf (get miro2_msg::priority_peak :definition-)
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

int32 stream_index
float32[2] loc_d
float32 height
float32 size
float32 azim
float32 elev

float32 size_norm
float32 volume
float32 range

int32 actioned


")



(provide :miro2_msg/priority_peak "d2924675ffe77da16f66ce7eb1cbeb6d")


