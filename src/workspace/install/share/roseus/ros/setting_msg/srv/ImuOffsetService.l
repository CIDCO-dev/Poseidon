;; Auto-generated. Do not edit!


(when (boundp 'setting_msg::ImuOffsetService)
  (if (not (find-package "SETTING_MSG"))
    (make-package "SETTING_MSG"))
  (shadow 'ImuOffsetService (find-package "SETTING_MSG")))
(unless (find-package "SETTING_MSG::IMUOFFSETSERVICE")
  (make-package "SETTING_MSG::IMUOFFSETSERVICE"))
(unless (find-package "SETTING_MSG::IMUOFFSETSERVICEREQUEST")
  (make-package "SETTING_MSG::IMUOFFSETSERVICEREQUEST"))
(unless (find-package "SETTING_MSG::IMUOFFSETSERVICERESPONSE")
  (make-package "SETTING_MSG::IMUOFFSETSERVICERESPONSE"))

(in-package "ROS")





(defclass setting_msg::ImuOffsetServiceRequest
  :super ros::object
  :slots (_headingOffset _pitchOffset _rollOffset ))

(defmethod setting_msg::ImuOffsetServiceRequest
  (:init
   (&key
    ((:headingOffset __headingOffset) 0.0)
    ((:pitchOffset __pitchOffset) 0.0)
    ((:rollOffset __rollOffset) 0.0)
    )
   (send-super :init)
   (setq _headingOffset (float __headingOffset))
   (setq _pitchOffset (float __pitchOffset))
   (setq _rollOffset (float __rollOffset))
   self)
  (:headingOffset
   (&optional __headingOffset)
   (if __headingOffset (setq _headingOffset __headingOffset)) _headingOffset)
  (:pitchOffset
   (&optional __pitchOffset)
   (if __pitchOffset (setq _pitchOffset __pitchOffset)) _pitchOffset)
  (:rollOffset
   (&optional __rollOffset)
   (if __rollOffset (setq _rollOffset __rollOffset)) _rollOffset)
  (:serialization-length
   ()
   (+
    ;; float64 _headingOffset
    8
    ;; float64 _pitchOffset
    8
    ;; float64 _rollOffset
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _headingOffset
       (sys::poke _headingOffset (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _pitchOffset
       (sys::poke _pitchOffset (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _rollOffset
       (sys::poke _rollOffset (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _headingOffset
     (setq _headingOffset (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _pitchOffset
     (setq _pitchOffset (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _rollOffset
     (setq _rollOffset (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass setting_msg::ImuOffsetServiceResponse
  :super ros::object
  :slots (_value ))

(defmethod setting_msg::ImuOffsetServiceResponse
  (:init
   (&key
    ((:value __value) "")
    )
   (send-super :init)
   (setq _value (string __value))
   self)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:serialization-length
   ()
   (+
    ;; string _value
    4 (length _value)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _value
       (write-long (length _value) s) (princ _value s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _value
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _value (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass setting_msg::ImuOffsetService
  :super ros::object
  :slots ())

(setf (get setting_msg::ImuOffsetService :md5sum-) "f17e28b74ba2da00db559b27e3cb5a7f")
(setf (get setting_msg::ImuOffsetService :datatype-) "setting_msg/ImuOffsetService")
(setf (get setting_msg::ImuOffsetService :request) setting_msg::ImuOffsetServiceRequest)
(setf (get setting_msg::ImuOffsetService :response) setting_msg::ImuOffsetServiceResponse)

(defmethod setting_msg::ImuOffsetServiceRequest
  (:response () (instance setting_msg::ImuOffsetServiceResponse :init)))

(setf (get setting_msg::ImuOffsetServiceRequest :md5sum-) "f17e28b74ba2da00db559b27e3cb5a7f")
(setf (get setting_msg::ImuOffsetServiceRequest :datatype-) "setting_msg/ImuOffsetServiceRequest")
(setf (get setting_msg::ImuOffsetServiceRequest :definition-)
      "float64 headingOffset
float64 pitchOffset
float64 rollOffset
---
string value

")

(setf (get setting_msg::ImuOffsetServiceResponse :md5sum-) "f17e28b74ba2da00db559b27e3cb5a7f")
(setf (get setting_msg::ImuOffsetServiceResponse :datatype-) "setting_msg/ImuOffsetServiceResponse")
(setf (get setting_msg::ImuOffsetServiceResponse :definition-)
      "float64 headingOffset
float64 pitchOffset
float64 rollOffset
---
string value

")



(provide :setting_msg/ImuOffsetService "f17e28b74ba2da00db559b27e3cb5a7f")


