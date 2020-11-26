
(cl:in-package :asdf)

(defsystem "raspberrypi_vitals_msg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "sysinfo" :depends-on ("_package_sysinfo"))
    (:file "_package_sysinfo" :depends-on ("_package"))
  ))