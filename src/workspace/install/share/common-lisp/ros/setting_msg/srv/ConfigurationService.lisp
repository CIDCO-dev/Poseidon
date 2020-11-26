; Auto-generated. Do not edit!


(cl:in-package setting_msg-srv)


;//! \htmlinclude ConfigurationService-request.msg.html

(cl:defclass <ConfigurationService-request> (roslisp-msg-protocol:ros-message)
  ((key
    :reader key
    :initarg :key
    :type cl:string
    :initform ""))
)

(cl:defclass ConfigurationService-request (<ConfigurationService-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConfigurationService-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConfigurationService-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name setting_msg-srv:<ConfigurationService-request> is deprecated: use setting_msg-srv:ConfigurationService-request instead.")))

(cl:ensure-generic-function 'key-val :lambda-list '(m))
(cl:defmethod key-val ((m <ConfigurationService-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader setting_msg-srv:key-val is deprecated.  Use setting_msg-srv:key instead.")
  (key m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConfigurationService-request>) ostream)
  "Serializes a message object of type '<ConfigurationService-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'key))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'key))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConfigurationService-request>) istream)
  "Deserializes a message object of type '<ConfigurationService-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'key) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'key) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConfigurationService-request>)))
  "Returns string type for a service object of type '<ConfigurationService-request>"
  "setting_msg/ConfigurationServiceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConfigurationService-request)))
  "Returns string type for a service object of type 'ConfigurationService-request"
  "setting_msg/ConfigurationServiceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConfigurationService-request>)))
  "Returns md5sum for a message object of type '<ConfigurationService-request>"
  "c7428de72b961d5dc25117ffa14882f8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConfigurationService-request)))
  "Returns md5sum for a message object of type 'ConfigurationService-request"
  "c7428de72b961d5dc25117ffa14882f8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConfigurationService-request>)))
  "Returns full string definition for message of type '<ConfigurationService-request>"
  (cl:format cl:nil "string key~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConfigurationService-request)))
  "Returns full string definition for message of type 'ConfigurationService-request"
  (cl:format cl:nil "string key~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConfigurationService-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'key))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConfigurationService-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ConfigurationService-request
    (cl:cons ':key (key msg))
))
;//! \htmlinclude ConfigurationService-response.msg.html

(cl:defclass <ConfigurationService-response> (roslisp-msg-protocol:ros-message)
  ((value
    :reader value
    :initarg :value
    :type cl:string
    :initform ""))
)

(cl:defclass ConfigurationService-response (<ConfigurationService-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConfigurationService-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConfigurationService-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name setting_msg-srv:<ConfigurationService-response> is deprecated: use setting_msg-srv:ConfigurationService-response instead.")))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <ConfigurationService-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader setting_msg-srv:value-val is deprecated.  Use setting_msg-srv:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConfigurationService-response>) ostream)
  "Serializes a message object of type '<ConfigurationService-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'value))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConfigurationService-response>) istream)
  "Deserializes a message object of type '<ConfigurationService-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'value) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'value) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConfigurationService-response>)))
  "Returns string type for a service object of type '<ConfigurationService-response>"
  "setting_msg/ConfigurationServiceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConfigurationService-response)))
  "Returns string type for a service object of type 'ConfigurationService-response"
  "setting_msg/ConfigurationServiceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConfigurationService-response>)))
  "Returns md5sum for a message object of type '<ConfigurationService-response>"
  "c7428de72b961d5dc25117ffa14882f8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConfigurationService-response)))
  "Returns md5sum for a message object of type 'ConfigurationService-response"
  "c7428de72b961d5dc25117ffa14882f8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConfigurationService-response>)))
  "Returns full string definition for message of type '<ConfigurationService-response>"
  (cl:format cl:nil "string value~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConfigurationService-response)))
  "Returns full string definition for message of type 'ConfigurationService-response"
  (cl:format cl:nil "string value~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConfigurationService-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'value))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConfigurationService-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ConfigurationService-response
    (cl:cons ':value (value msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ConfigurationService)))
  'ConfigurationService-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ConfigurationService)))
  'ConfigurationService-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConfigurationService)))
  "Returns string type for a service object of type '<ConfigurationService>"
  "setting_msg/ConfigurationService")