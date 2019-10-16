;; Auto-generated. Do not edit!


(when (boundp 'miro2_msg::object_tag)
  (if (not (find-package "MIRO2_MSG"))
    (make-package "MIRO2_MSG"))
  (shadow 'object_tag (find-package "MIRO2_MSG")))
(unless (find-package "MIRO2_MSG::OBJECT_TAG")
  (make-package "MIRO2_MSG::OBJECT_TAG"))

(in-package "ROS")
;;//! \htmlinclude object_tag.msg.html


(defclass miro2_msg::object_tag
  :super ros::object
  :slots (_conf _id _centre _corners ))

(defmethod miro2_msg::object_tag
  (:init
   (&key
    ((:conf __conf) 0.0)
    ((:id __id) 0)
    ((:centre __centre) (make-array 2 :initial-element 0.0 :element-type :float))
    ((:corners __corners) (make-array 8 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _conf (float __conf))
   (setq _id (round __id))
   (setq _centre __centre)
   (setq _corners __corners)
   self)
  (:conf
   (&optional __conf)
   (if __conf (setq _conf __conf)) _conf)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:centre
   (&optional __centre)
   (if __centre (setq _centre __centre)) _centre)
  (:corners
   (&optional __corners)
   (if __corners (setq _corners __corners)) _corners)
  (:serialization-length
   ()
   (+
    ;; float32 _conf
    4
    ;; int32 _id
    4
    ;; float32[2] _centre
    (* 4    2)
    ;; float32[8] _corners
    (* 4    8)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _conf
       (sys::poke _conf (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _id
       (write-long _id s)
     ;; float32[2] _centre
     (dotimes (i 2)
       (sys::poke (elt _centre i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[8] _corners
     (dotimes (i 8)
       (sys::poke (elt _corners i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _conf
     (setq _conf (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32[2] _centre
   (dotimes (i (length _centre))
     (setf (elt _centre i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[8] _corners
   (dotimes (i (length _corners))
     (setf (elt _corners i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;;
   self)
  )

(setf (get miro2_msg::object_tag :md5sum-) "acca63b10aeb84130dcb05309da8749a")
(setf (get miro2_msg::object_tag :datatype-) "miro2_msg/object_tag")
(setf (get miro2_msg::object_tag :definition-)
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
int32 id
float32[2] centre
float32[8] corners


")



(provide :miro2_msg/object_tag "acca63b10aeb84130dcb05309da8749a")


