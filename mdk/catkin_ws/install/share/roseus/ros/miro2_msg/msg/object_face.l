;; Auto-generated. Do not edit!


(when (boundp 'miro2_msg::object_face)
  (if (not (find-package "MIRO2_MSG"))
    (make-package "MIRO2_MSG"))
  (shadow 'object_face (find-package "MIRO2_MSG")))
(unless (find-package "MIRO2_MSG::OBJECT_FACE")
  (make-package "MIRO2_MSG::OBJECT_FACE"))

(in-package "ROS")
;;//! \htmlinclude object_face.msg.html


(defclass miro2_msg::object_face
  :super ros::object
  :slots (_conf _corner _size ))

(defmethod miro2_msg::object_face
  (:init
   (&key
    ((:conf __conf) 0.0)
    ((:corner __corner) (make-array 2 :initial-element 0.0 :element-type :float))
    ((:size __size) (make-array 2 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _conf (float __conf))
   (setq _corner __corner)
   (setq _size __size)
   self)
  (:conf
   (&optional __conf)
   (if __conf (setq _conf __conf)) _conf)
  (:corner
   (&optional __corner)
   (if __corner (setq _corner __corner)) _corner)
  (:size
   (&optional __size)
   (if __size (setq _size __size)) _size)
  (:serialization-length
   ()
   (+
    ;; float32 _conf
    4
    ;; float32[2] _corner
    (* 4    2)
    ;; float32[2] _size
    (* 4    2)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _conf
       (sys::poke _conf (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32[2] _corner
     (dotimes (i 2)
       (sys::poke (elt _corner i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[2] _size
     (dotimes (i 2)
       (sys::poke (elt _size i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _conf
     (setq _conf (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32[2] _corner
   (dotimes (i (length _corner))
     (setf (elt _corner i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[2] _size
   (dotimes (i (length _size))
     (setf (elt _size i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;;
   self)
  )

(setf (get miro2_msg::object_face :md5sum-) "0608d5dd7e088755bafb29283da99f24")
(setf (get miro2_msg::object_face :datatype-) "miro2_msg/object_face")
(setf (get miro2_msg::object_face :definition-)
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

float32 conf
float32[2] corner
float32[2] size


")



(provide :miro2_msg/object_face "0608d5dd7e088755bafb29283da99f24")


