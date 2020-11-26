; Auto-generated. Do not edit!


(cl:in-package logger_service-srv)


;//! \htmlinclude ToggleLogging-request.msg.html

(cl:defclass <ToggleLogging-request> (roslisp-msg-protocol:ros-message)
  ((loggingEnabled
    :reader loggingEnabled
    :initarg :loggingEnabled
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ToggleLogging-request (<ToggleLogging-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ToggleLogging-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ToggleLogging-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name logger_service-srv:<ToggleLogging-request> is deprecated: use logger_service-srv:ToggleLogging-request instead.")))

(cl:ensure-generic-function 'loggingEnabled-val :lambda-list '(m))
(cl:defmethod loggingEnabled-val ((m <ToggleLogging-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader logger_service-srv:loggingEnabled-val is deprecated.  Use logger_service-srv:loggingEnabled instead.")
  (loggingEnabled m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ToggleLogging-request>) ostream)
  "Serializes a message object of type '<ToggleLogging-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'loggingEnabled) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ToggleLogging-request>) istream)
  "Deserializes a message object of type '<ToggleLogging-request>"
    (cl:setf (cl:slot-value msg 'loggingEnabled) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ToggleLogging-request>)))
  "Returns string type for a service object of type '<ToggleLogging-request>"
  "logger_service/ToggleLoggingRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ToggleLogging-request)))
  "Returns string type for a service object of type 'ToggleLogging-request"
  "logger_service/ToggleLoggingRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ToggleLogging-request>)))
  "Returns md5sum for a message object of type '<ToggleLogging-request>"
  "fecf66bb22f64ee8735af9d41647765f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ToggleLogging-request)))
  "Returns md5sum for a message object of type 'ToggleLogging-request"
  "fecf66bb22f64ee8735af9d41647765f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ToggleLogging-request>)))
  "Returns full string definition for message of type '<ToggleLogging-request>"
  (cl:format cl:nil "bool loggingEnabled~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ToggleLogging-request)))
  "Returns full string definition for message of type 'ToggleLogging-request"
  (cl:format cl:nil "bool loggingEnabled~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ToggleLogging-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ToggleLogging-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ToggleLogging-request
    (cl:cons ':loggingEnabled (loggingEnabled msg))
))
;//! \htmlinclude ToggleLogging-response.msg.html

(cl:defclass <ToggleLogging-response> (roslisp-msg-protocol:ros-message)
  ((loggingStatus
    :reader loggingStatus
    :initarg :loggingStatus
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ToggleLogging-response (<ToggleLogging-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ToggleLogging-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ToggleLogging-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name logger_service-srv:<ToggleLogging-response> is deprecated: use logger_service-srv:ToggleLogging-response instead.")))

(cl:ensure-generic-function 'loggingStatus-val :lambda-list '(m))
(cl:defmethod loggingStatus-val ((m <ToggleLogging-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader logger_service-srv:loggingStatus-val is deprecated.  Use logger_service-srv:loggingStatus instead.")
  (loggingStatus m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ToggleLogging-response>) ostream)
  "Serializes a message object of type '<ToggleLogging-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'loggingStatus) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ToggleLogging-response>) istream)
  "Deserializes a message object of type '<ToggleLogging-response>"
    (cl:setf (cl:slot-value msg 'loggingStatus) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ToggleLogging-response>)))
  "Returns string type for a service object of type '<ToggleLogging-response>"
  "logger_service/ToggleLoggingResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ToggleLogging-response)))
  "Returns string type for a service object of type 'ToggleLogging-response"
  "logger_service/ToggleLoggingResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ToggleLogging-response>)))
  "Returns md5sum for a message object of type '<ToggleLogging-response>"
  "fecf66bb22f64ee8735af9d41647765f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ToggleLogging-response)))
  "Returns md5sum for a message object of type 'ToggleLogging-response"
  "fecf66bb22f64ee8735af9d41647765f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ToggleLogging-response>)))
  "Returns full string definition for message of type '<ToggleLogging-response>"
  (cl:format cl:nil "bool loggingStatus~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ToggleLogging-response)))
  "Returns full string definition for message of type 'ToggleLogging-response"
  (cl:format cl:nil "bool loggingStatus~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ToggleLogging-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ToggleLogging-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ToggleLogging-response
    (cl:cons ':loggingStatus (loggingStatus msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ToggleLogging)))
  'ToggleLogging-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ToggleLogging)))
  'ToggleLogging-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ToggleLogging)))
  "Returns string type for a service object of type '<ToggleLogging>"
  "logger_service/ToggleLogging")