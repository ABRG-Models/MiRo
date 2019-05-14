;; Auto-generated. Do not edit!


(when (boundp 'miro2_msg::push)
  (if (not (find-package "MIRO2_MSG"))
    (make-package "MIRO2_MSG"))
  (shadow 'push (find-package "MIRO2_MSG")))
(unless (find-package "MIRO2_MSG::PUSH")
  (make-package "MIRO2_MSG::PUSH"))

(in-package "ROS")
;;//! \htmlinclude push.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass miro2_msg::push
  :super ros::object
  :slots (_link _flags _pushpos _pushvec ))

(defmethod miro2_msg::push
  (:init
   (&key
    ((:link __link) 0)
    ((:flags __flags) 0)
    ((:pushpos __pushpos) (instance geometry_msgs::Point :init))
    ((:pushvec __pushvec) (instance geometry_msgs::Vector3 :init))
    )
   (send-super :init)
   (setq _link (round __link))
   (setq _flags (round __flags))
   (setq _pushpos __pushpos)
   (setq _pushvec __pushvec)
   self)
  (:link
   (&optional __link)
   (if __link (setq _link __link)) _link)
  (:flags
   (&optional __flags)
   (if __flags (setq _flags __flags)) _flags)
  (:pushpos
   (&rest __pushpos)
   (if (keywordp (car __pushpos))
       (send* _pushpos __pushpos)
     (progn
       (if __pushpos (setq _pushpos (car __pushpos)))
       _pushpos)))
  (:pushvec
   (&rest __pushvec)
   (if (keywordp (car __pushvec))
       (send* _pushvec __pushvec)
     (progn
       (if __pushvec (setq _pushvec (car __pushvec)))
       _pushvec)))
  (:serialization-length
   ()
   (+
    ;; int32 _link
    4
    ;; uint32 _flags
    4
    ;; geometry_msgs/Point _pushpos
    (send _pushpos :serialization-length)
    ;; geometry_msgs/Vector3 _pushvec
    (send _pushvec :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _link
       (write-long _link s)
     ;; uint32 _flags
       (write-long _flags s)
     ;; geometry_msgs/Point _pushpos
       (send _pushpos :serialize s)
     ;; geometry_msgs/Vector3 _pushvec
       (send _pushvec :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _link
     (setq _link (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _flags
     (setq _flags (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; geometry_msgs/Point _pushpos
     (send _pushpos :deserialize buf ptr-) (incf ptr- (send _pushpos :serialization-length))
   ;; geometry_msgs/Vector3 _pushvec
     (send _pushvec :deserialize buf ptr-) (incf ptr- (send _pushvec :serialization-length))
   ;;
   self)
  )

(setf (get miro2_msg::push :md5sum-) "6271e5b7c8f54208b938d70e5eaafbe6")
(setf (get miro2_msg::push :datatype-) "miro2_msg/push")
(setf (get miro2_msg::push :definition-)
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
#	Description... TODO.

#	The identifier of the pushed link
int32 link

#	DOCLINK PUSH FLAGS
#
#	The values of these flags are defined in miro2.constants
#
#	PUSH_FLAG_IMPULSE
#		Treat push as an impulse (in mm).
#
#	PUSH_FLAG_VELOCITY
#		Treat push as a velocity (in mm/sec).
#
#	NB: If neither of the above flags is set, the push should
#	not be actioned at all (this condition is used in server
#	to indicate \"not pending\").
#
#	PUSH_FLAG_NO_TRANSLATION
#		Zero out any resultant change in pose.dr (i.e. only
#		turn on the spot).
#
#	PUSH_FLAG_NO_ROTATION
#		Zero out any resultant change in pose.dtheta /and/
#		in pose.dr (i.e. do not move wheels at all).
#
#	PUSH_FLAG_NO_NECK_MOVEMENT
#		Zero out any resultant change in neck configuration;
#		This flag is independent of NO_TRANSLATION/ROTATION.
#
#	PUSH_FLAG_WAIT
#		Cause the push processor to wait for further pushes
#		before publishing a velocity at its output. This is
#		required if you want to pass multiple push streams.
uint32 flags

#	The pushed point, in the reference frame of the pushed link
geometry_msgs/Point pushpos

#	The push vector, in the reference frame of the pushed link
geometry_msgs/Vector3 pushvec





================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :miro2_msg/push "6271e5b7c8f54208b938d70e5eaafbe6")


