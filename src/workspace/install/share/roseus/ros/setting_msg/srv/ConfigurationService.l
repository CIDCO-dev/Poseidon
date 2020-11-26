;; Auto-generated. Do not edit!


(when (boundp 'setting_msg::ConfigurationService)
  (if (not (find-package "SETTING_MSG"))
    (make-package "SETTING_MSG"))
  (shadow 'ConfigurationService (find-package "SETTING_MSG")))
(unless (find-package "SETTING_MSG::CONFIGURATIONSERVICE")
  (make-package "SETTING_MSG::CONFIGURATIONSERVICE"))
(unless (find-package "SETTING_MSG::CONFIGURATIONSERVICEREQUEST")
  (make-package "SETTING_MSG::CONFIGURATIONSERVICEREQUEST"))
(unless (find-package "SETTING_MSG::CONFIGURATIONSERVICERESPONSE")
  (make-package "SETTING_MSG::CONFIGURATIONSERVICERESPONSE"))

(in-package "ROS")





(defclass setting_msg::ConfigurationServiceRequest
  :super ros::object
  :slots (_key ))

(defmethod setting_msg::ConfigurationServiceRequest
  (:init
   (&key
    ((:key __key) "")
    )
   (send-super :init)
   (setq _key (string __key))
   self)
  (:key
   (&optional __key)
   (if __key (setq _key __key)) _key)
  (:serialization-length
   ()
   (+
    ;; string _key
    4 (length _key)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _key
       (write-long (length _key) s) (princ _key s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _key
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _key (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass setting_msg::ConfigurationServiceResponse
  :super ros::object
  :slots (_value ))

(defmethod setting_msg::ConfigurationServiceResponse
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

(defclass setting_msg::ConfigurationService
  :super ros::object
  :slots ())

(setf (get setting_msg::ConfigurationService :md5sum-) "c7428de72b961d5dc25117ffa14882f8")
(setf (get setting_msg::ConfigurationService :datatype-) "setting_msg/ConfigurationService")
(setf (get setting_msg::ConfigurationService :request) setting_msg::ConfigurationServiceRequest)
(setf (get setting_msg::ConfigurationService :response) setting_msg::ConfigurationServiceResponse)

(defmethod setting_msg::ConfigurationServiceRequest
  (:response () (instance setting_msg::ConfigurationServiceResponse :init)))

(setf (get setting_msg::ConfigurationServiceRequest :md5sum-) "c7428de72b961d5dc25117ffa14882f8")
(setf (get setting_msg::ConfigurationServiceRequest :datatype-) "setting_msg/ConfigurationServiceRequest")
(setf (get setting_msg::ConfigurationServiceRequest :definition-)
      "string key
---
string value

")

(setf (get setting_msg::ConfigurationServiceResponse :md5sum-) "c7428de72b961d5dc25117ffa14882f8")
(setf (get setting_msg::ConfigurationServiceResponse :datatype-) "setting_msg/ConfigurationServiceResponse")
(setf (get setting_msg::ConfigurationServiceResponse :definition-)
      "string key
---
string value

")



(provide :setting_msg/ConfigurationService "c7428de72b961d5dc25117ffa14882f8")


