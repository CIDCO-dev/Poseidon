
(cl:in-package :asdf)

(defsystem "state_controller_msg-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :state_controller_msg-msg
)
  :components ((:file "_package")
    (:file "GetStateService" :depends-on ("_package_GetStateService"))
    (:file "_package_GetStateService" :depends-on ("_package"))
  ))