;; Auto-generated. Do not edit!


(when (boundp 'miro2_msg::objects)
  (if (not (find-package "MIRO2_MSG"))
    (make-package "MIRO2_MSG"))
  (shadow 'objects (find-package "MIRO2_MSG")))
(unless (find-package "MIRO2_MSG::OBJECTS")
  (make-package "MIRO2_MSG::OBJECTS"))

(in-package "ROS")
;;//! \htmlinclude objects.msg.html


(defclass miro2_msg::objects
  :super ros::object
  :slots (_stream_index _tags _balls _faces ))

(defmethod miro2_msg::objects
  (:init
   (&key
    ((:stream_index __stream_index) 0)
    ((:tags __tags) (let (r) (dotimes (i 0) (push (instance miro2_msg::object_tag :init) r)) r))
    ((:balls __balls) (let (r) (dotimes (i 0) (push (instance miro2_msg::object_ball :init) r)) r))
    ((:faces __faces) (let (r) (dotimes (i 0) (push (instance miro2_msg::object_face :init) r)) r))
    )
   (send-super :init)
   (setq _stream_index (round __stream_index))
   (setq _tags __tags)
   (setq _balls __balls)
   (setq _faces __faces)
   self)
  (:stream_index
   (&optional __stream_index)
   (if __stream_index (setq _stream_index __stream_index)) _stream_index)
  (:tags
   (&rest __tags)
   (if (keywordp (car __tags))
       (send* _tags __tags)
     (progn
       (if __tags (setq _tags (car __tags)))
       _tags)))
  (:balls
   (&rest __balls)
   (if (keywordp (car __balls))
       (send* _balls __balls)
     (progn
       (if __balls (setq _balls (car __balls)))
       _balls)))
  (:faces
   (&rest __faces)
   (if (keywordp (car __faces))
       (send* _faces __faces)
     (progn
       (if __faces (setq _faces (car __faces)))
       _faces)))
  (:serialization-length
   ()
   (+
    ;; int32 _stream_index
    4
    ;; miro2_msg/object_tag[] _tags
    (apply #'+ (send-all _tags :serialization-length)) 4
    ;; miro2_msg/object_ball[] _balls
    (apply #'+ (send-all _balls :serialization-length)) 4
    ;; miro2_msg/object_face[] _faces
    (apply #'+ (send-all _faces :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _stream_index
       (write-long _stream_index s)
     ;; miro2_msg/object_tag[] _tags
     (write-long (length _tags) s)
     (dolist (elem _tags)
       (send elem :serialize s)
       )
     ;; miro2_msg/object_ball[] _balls
     (write-long (length _balls) s)
     (dolist (elem _balls)
       (send elem :serialize s)
       )
     ;; miro2_msg/object_face[] _faces
     (write-long (length _faces) s)
     (dolist (elem _faces)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _stream_index
     (setq _stream_index (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; miro2_msg/object_tag[] _tags
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _tags (let (r) (dotimes (i n) (push (instance miro2_msg::object_tag :init) r)) r))
     (dolist (elem- _tags)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; miro2_msg/object_ball[] _balls
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _balls (let (r) (dotimes (i n) (push (instance miro2_msg::object_ball :init) r)) r))
     (dolist (elem- _balls)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; miro2_msg/object_face[] _faces
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _faces (let (r) (dotimes (i n) (push (instance miro2_msg::object_face :init) r)) r))
     (dolist (elem- _faces)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get miro2_msg::objects :md5sum-) "0751afb21e15f80b448531e265b3963f")
(setf (get miro2_msg::objects :datatype-) "miro2_msg/objects")
(setf (get miro2_msg::objects :definition-)
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

# header
int32 stream_index

#	objects
object_tag[] tags
object_ball[] balls
object_face[] faces


================================================================================
MSG: miro2_msg/object_tag
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

float32 conf
int32 id
float32[2] centre
float32[8] corners


================================================================================
MSG: miro2_msg/object_ball
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

float32 conf
float32[2] centre
float32 radius


================================================================================
MSG: miro2_msg/object_face
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

float32 conf
float32[2] corner
float32[2] size


")



(provide :miro2_msg/objects "0751afb21e15f80b448531e265b3963f")


