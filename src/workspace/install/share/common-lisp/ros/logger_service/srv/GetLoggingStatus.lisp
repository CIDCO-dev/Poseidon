; Auto-generated. Do not edit!


(cl:in-package logger_service-srv)


;//! \htmlinclude GetLoggingStatus-request.msg.html

(cl:defclass <GetLoggingStatus-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetLoggingStatus-request (<GetLoggingStatus-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetLoggingStatus-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetLoggingStatus-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name logger_service-srv:<GetLoggingStatus-request> is deprecated: use logger_service-srv:GetLoggingStatus-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetLoggingStatus-request>) ostream)
  "Serializes a message object of type '<GetLoggingStatus-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetLoggingStatus-request>) istream)
  "Deserializes a message object of type '<GetLoggingStatus-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetLoggingStatus-request>)))
  "Returns string type for a service object of type '<GetLoggingStatus-request>"
  "logger_service/GetLoggingStatusRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetLoggingStatus-request)))
  "Returns string type for a service object of type 'GetLoggingStatus-request"
  "logger_service/GetLoggingStatusRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetLoggingStatus-request>)))
  "Returns md5sum for a message object of type '<GetLoggingStatus-request>"
  "3a1255d4d998bd4d6585c64639b5ee9a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetLoggingStatus-request)))
  "Returns md5sum for a message object of type 'GetLoggingStatus-request"
  "3a1255d4d998bd4d6585c64639b5ee9a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetLoggingStatus-request>)))
  "Returns full string definition for message of type '<GetLoggingStatus-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetLoggingStatus-request)))
  "Returns full string definition for message of type 'GetLoggingStatus-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetLoggingStatus-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetLoggingStatus-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetLoggingStatus-request
))
;//! \htmlinclude GetLoggingStatus-response.msg.html

(cl:defclass <GetLoggingStatus-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GetLoggingStatus-response (<GetLoggingStatus-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetLoggingStatus-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetLoggingStatus-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name logger_service-srv:<GetLoggingStatus-response> is deprecated: use logger_service-srv:GetLoggingStatus-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <GetLoggingStatus-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader logger_service-srv:status-val is deprecated.  Use logger_service-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetLoggingStatus-response>) ostream)
  "Serializes a message object of type '<GetLoggingStatus-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'status) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetLoggingStatus-response>) istream)
  "Deserializes a message object of type '<GetLoggingStatus-response>"
    (cl:setf (cl:slot-value msg 'status) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetLoggingStatus-response>)))
  "Returns string type for a service object of type '<GetLoggingStatus-response>"
  "logger_service/GetLoggingStatusResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetLoggingStatus-response)))
  "Returns string type for a service object of type 'GetLoggingStatus-response"
  "logger_service/GetLoggingStatusResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetLoggingStatus-response>)))
  "Returns md5sum for a message object of type '<GetLoggingStatus-response>"
  "3a1255d4d998bd4d6585c64639b5ee9a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetLoggingStatus-response)))
  "Returns md5sum for a message object of type 'GetLoggingStatus-response"
  "3a1255d4d998bd4d6585c64639b5ee9a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetLoggingStatus-response>)))
  "Returns full string definition for message of type '<GetLoggingStatus-response>"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetLoggingStatus-response)))
  "Returns full string definition for message of type 'GetLoggingStatus-response"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetLoggingStatus-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetLoggingStatus-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetLoggingStatus-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetLoggingStatus)))
  'GetLoggingStatus-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetLoggingStatus)))
  'GetLoggingStatus-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetLoggingStatus)))
  "Returns string type for a service object of type '<GetLoggingStatus>"
  "logger_service/GetLoggingStatus")