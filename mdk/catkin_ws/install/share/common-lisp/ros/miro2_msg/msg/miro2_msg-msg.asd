
(cl:in-package :asdf)

(defsystem "miro2_msg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :nav_msgs-msg
               :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "affect" :depends-on ("_package_affect"))
    (:file "_package_affect" :depends-on ("_package"))
    (:file "affect_state" :depends-on ("_package_affect_state"))
    (:file "_package_affect_state" :depends-on ("_package"))
    (:file "push" :depends-on ("_package_push"))
    (:file "_package_push" :depends-on ("_package"))
    (:file "sensors_package" :depends-on ("_package_sensors_package"))
    (:file "_package_sensors_package" :depends-on ("_package"))
    (:file "sleep" :depends-on ("_package_sleep"))
    (:file "_package_sleep" :depends-on ("_package"))
    (:file "voice_state" :depends-on ("_package_voice_state"))
    (:file "_package_voice_state" :depends-on ("_package"))
  ))