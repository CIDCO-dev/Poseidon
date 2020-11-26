
(cl:in-package :asdf)

(defsystem "state_controller_msg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :raspberrypi_vitals_msg-msg
               :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "State" :depends-on ("_package_State"))
    (:file "_package_State" :depends-on ("_package"))
  ))