;; Auto-generated. Do not edit!


(when (boundp 'logger_service::ToggleLogging)
  (if (not (find-package "LOGGER_SERVICE"))
    (make-package "LOGGER_SERVICE"))
  (shadow 'ToggleLogging (find-package "LOGGER_SERVICE")))
(unless (find-package "LOGGER_SERVICE::TOGGLELOGGING")
  (make-package "LOGGER_SERVICE::TOGGLELOGGING"))
(unless (find-package "LOGGER_SERVICE::TOGGLELOGGINGREQUEST")
  (make-package "LOGGER_SERVICE::TOGGLELOGGINGREQUEST"))
(unless (find-package "LOGGER_SERVICE::TOGGLELOGGINGRESPONSE")
  (make-package "LOGGER_SERVICE::TOGGLELOGGINGRESPONSE"))

(in-package "ROS")





(defclass logger_service::ToggleLoggingRequest
  :super ros::object
  :slots (_loggingEnabled ))

(defmethod logger_service::ToggleLoggingRequest
  (:init
   (&key
    ((:loggingEnabled __loggingEnabled) nil)
    )
   (send-super :init)
   (setq _loggingEnabled __loggingEnabled)
   self)
  (:loggingEnabled
   (&optional __loggingEnabled)
   (if __loggingEnabled (setq _loggingEnabled __loggingEnabled)) _loggingEnabled)
  (:serialization-length
   ()
   (+
    ;; bool _loggingEnabled
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _loggingEnabled
       (if _loggingEnabled (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _loggingEnabled
     (setq _loggingEnabled (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass logger_service::ToggleLoggingResponse
  :super ros::object
  :slots (_loggingStatus ))

(defmethod logger_service::ToggleLoggingResponse
  (:init
   (&key
    ((:loggingStatus __loggingStatus) nil)
    )
   (send-super :init)
   (setq _loggingStatus __loggingStatus)
   self)
  (:loggingStatus
   (&optional __loggingStatus)
   (if __loggingStatus (setq _loggingStatus __loggingStatus)) _loggingStatus)
  (:serialization-length
   ()
   (+
    ;; bool _loggingStatus
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _loggingStatus
       (if _loggingStatus (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _loggingStatus
     (setq _loggingStatus (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass logger_service::ToggleLogging
  :super ros::object
  :slots ())

(setf (get logger_service::ToggleLogging :md5sum-) "fecf66bb22f64ee8735af9d41647765f")
(setf (get logger_service::ToggleLogging :datatype-) "logger_service/ToggleLogging")
(setf (get logger_service::ToggleLogging :request) logger_service::ToggleLoggingRequest)
(setf (get logger_service::ToggleLogging :response) logger_service::ToggleLoggingResponse)

(defmethod logger_service::ToggleLoggingRequest
  (:response () (instance logger_service::ToggleLoggingResponse :init)))

(setf (get logger_service::ToggleLoggingRequest :md5sum-) "fecf66bb22f64ee8735af9d41647765f")
(setf (get logger_service::ToggleLoggingRequest :datatype-) "logger_service/ToggleLoggingRequest")
(setf (get logger_service::ToggleLoggingRequest :definition-)
      "bool loggingEnabled
---
bool loggingStatus

")

(setf (get logger_service::ToggleLoggingResponse :md5sum-) "fecf66bb22f64ee8735af9d41647765f")
(setf (get logger_service::ToggleLoggingResponse :datatype-) "logger_service/ToggleLoggingResponse")
(setf (get logger_service::ToggleLoggingResponse :definition-)
      "bool loggingEnabled
---
bool loggingStatus

")



(provide :logger_service/ToggleLogging "fecf66bb22f64ee8735af9d41647765f")


