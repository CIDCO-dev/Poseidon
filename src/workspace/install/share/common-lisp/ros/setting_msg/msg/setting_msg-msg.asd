
(cl:in-package :asdf)

(defsystem "setting_msg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Setting" :depends-on ("_package_Setting"))
    (:file "_package_Setting" :depends-on ("_package"))
  ))