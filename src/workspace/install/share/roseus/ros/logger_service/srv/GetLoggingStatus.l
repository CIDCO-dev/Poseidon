;; Auto-generated. Do not edit!


(when (boundp 'logger_service::GetLoggingStatus)
  (if (not (find-package "LOGGER_SERVICE"))
    (make-package "LOGGER_SERVICE"))
  (shadow 'GetLoggingStatus (find-package "LOGGER_SERVICE")))
(unless (find-package "LOGGER_SERVICE::GETLOGGINGSTATUS")
  (make-package "LOGGER_SERVICE::GETLOGGINGSTATUS"))
(unless (find-package "LOGGER_SERVICE::GETLOGGINGSTATUSREQUEST")
  (make-package "LOGGER_SERVICE::GETLOGGINGSTATUSREQUEST"))
(unless (find-package "LOGGER_SERVICE::GETLOGGINGSTATUSRESPONSE")
  (make-package "LOGGER_SERVICE::GETLOGGINGSTATUSRESPONSE"))

(in-package "ROS")





(defclass logger_service::GetLoggingStatusRequest
  :super ros::object
  :slots ())

(defmethod logger_service::GetLoggingStatusRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass logger_service::GetLoggingStatusResponse
  :super ros::object
  :slots (_status ))

(defmethod logger_service::GetLoggingStatusResponse
  (:init
   (&key
    ((:status __status) nil)
    )
   (send-super :init)
   (setq _status __status)
   self)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:serialization-length
   ()
   (+
    ;; bool _status
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _status
       (if _status (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _status
     (setq _status (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass logger_service::GetLoggingStatus
  :super ros::object
  :slots ())

(setf (get logger_service::GetLoggingStatus :md5sum-) "3a1255d4d998bd4d6585c64639b5ee9a")
(setf (get logger_service::GetLoggingStatus :datatype-) "logger_service/GetLoggingStatus")
(setf (get logger_service::GetLoggingStatus :request) logger_service::GetLoggingStatusRequest)
(setf (get logger_service::GetLoggingStatus :response) logger_service::GetLoggingStatusResponse)

(defmethod logger_service::GetLoggingStatusRequest
  (:response () (instance logger_service::GetLoggingStatusResponse :init)))

(setf (get logger_service::GetLoggingStatusRequest :md5sum-) "3a1255d4d998bd4d6585c64639b5ee9a")
(setf (get logger_service::GetLoggingStatusRequest :datatype-) "logger_service/GetLoggingStatusRequest")
(setf (get logger_service::GetLoggingStatusRequest :definition-)
      "---
bool status

")

(setf (get logger_service::GetLoggingStatusResponse :md5sum-) "3a1255d4d998bd4d6585c64639b5ee9a")
(setf (get logger_service::GetLoggingStatusResponse :datatype-) "logger_service/GetLoggingStatusResponse")
(setf (get logger_service::GetLoggingStatusResponse :definition-)
      "---
bool status

")



(provide :logger_service/GetLoggingStatus "3a1255d4d998bd4d6585c64639b5ee9a")


