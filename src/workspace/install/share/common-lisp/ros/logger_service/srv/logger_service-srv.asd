
(cl:in-package :asdf)

(defsystem "logger_service-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "GetLoggingStatus" :depends-on ("_package_GetLoggingStatus"))
    (:file "_package_GetLoggingStatus" :depends-on ("_package"))
    (:file "ToggleLogging" :depends-on ("_package_ToggleLogging"))
    (:file "_package_ToggleLogging" :depends-on ("_package"))
  ))