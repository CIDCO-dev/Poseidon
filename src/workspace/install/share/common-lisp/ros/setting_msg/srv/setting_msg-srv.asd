
(cl:in-package :asdf)

(defsystem "setting_msg-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ConfigurationService" :depends-on ("_package_ConfigurationService"))
    (:file "_package_ConfigurationService" :depends-on ("_package"))
    (:file "ImuOffsetService" :depends-on ("_package_ImuOffsetService"))
    (:file "_package_ImuOffsetService" :depends-on ("_package"))
  ))