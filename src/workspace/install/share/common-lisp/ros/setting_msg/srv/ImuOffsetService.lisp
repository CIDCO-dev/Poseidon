; Auto-generated. Do not edit!


(cl:in-package setting_msg-srv)


;//! \htmlinclude ImuOffsetService-request.msg.html

(cl:defclass <ImuOffsetService-request> (roslisp-msg-protocol:ros-message)
  ((headingOffset
    :reader headingOffset
    :initarg :headingOffset
    :type cl:float
    :initform 0.0)
   (pitchOffset
    :reader pitchOffset
    :initarg :pitchOffset
    :type cl:float
    :initform 0.0)
   (rollOffset
    :reader rollOffset
    :initarg :rollOffset
    :type cl:float
    :initform 0.0))
)

(cl:defclass ImuOffsetService-request (<ImuOffsetService-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ImuOffsetService-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ImuOffsetService-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name setting_msg-srv:<ImuOffsetService-request> is deprecated: use setting_msg-srv:ImuOffsetService-request instead.")))

(cl:ensure-generic-function 'headingOffset-val :lambda-list '(m))
(cl:defmethod headingOffset-val ((m <ImuOffsetService-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader setting_msg-srv:headingOffset-val is deprecated.  Use setting_msg-srv:headingOffset instead.")
  (headingOffset m))

(cl:ensure-generic-function 'pitchOffset-val :lambda-list '(m))
(cl:defmethod pitchOffset-val ((m <ImuOffsetService-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader setting_msg-srv:pitchOffset-val is deprecated.  Use setting_msg-srv:pitchOffset instead.")
  (pitchOffset m))

(cl:ensure-generic-function 'rollOffset-val :lambda-list '(m))
(cl:defmethod rollOffset-val ((m <ImuOffsetService-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader setting_msg-srv:rollOffset-val is deprecated.  Use setting_msg-srv:rollOffset instead.")
  (rollOffset m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ImuOffsetService-request>) ostream)
  "Serializes a message object of type '<ImuOffsetService-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'headingOffset))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'pitchOffset))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'rollOffset))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ImuOffsetService-request>) istream)
  "Deserializes a message object of type '<ImuOffsetService-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'headingOffset) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pitchOffset) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rollOffset) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ImuOffsetService-request>)))
  "Returns string type for a service object of type '<ImuOffsetService-request>"
  "setting_msg/ImuOffsetServiceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ImuOffsetService-request)))
  "Returns string type for a service object of type 'ImuOffsetService-request"
  "setting_msg/ImuOffsetServiceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ImuOffsetService-request>)))
  "Returns md5sum for a message object of type '<ImuOffsetService-request>"
  "f17e28b74ba2da00db559b27e3cb5a7f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ImuOffsetService-request)))
  "Returns md5sum for a message object of type 'ImuOffsetService-request"
  "f17e28b74ba2da00db559b27e3cb5a7f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ImuOffsetService-request>)))
  "Returns full string definition for message of type '<ImuOffsetService-request>"
  (cl:format cl:nil "float64 headingOffset~%float64 pitchOffset~%float64 rollOffset~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ImuOffsetService-request)))
  "Returns full string definition for message of type 'ImuOffsetService-request"
  (cl:format cl:nil "float64 headingOffset~%float64 pitchOffset~%float64 rollOffset~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ImuOffsetService-request>))
  (cl:+ 0
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ImuOffsetService-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ImuOffsetService-request
    (cl:cons ':headingOffset (headingOffset msg))
    (cl:cons ':pitchOffset (pitchOffset msg))
    (cl:cons ':rollOffset (rollOffset msg))
))
;//! \htmlinclude ImuOffsetService-response.msg.html

(cl:defclass <ImuOffsetService-response> (roslisp-msg-protocol:ros-message)
  ((value
    :reader value
    :initarg :value
    :type cl:string
    :initform ""))
)

(cl:defclass ImuOffsetService-response (<ImuOffsetService-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ImuOffsetService-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ImuOffsetService-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name setting_msg-srv:<ImuOffsetService-response> is deprecated: use setting_msg-srv:ImuOffsetService-response instead.")))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <ImuOffsetService-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader setting_msg-srv:value-val is deprecated.  Use setting_msg-srv:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ImuOffsetService-response>) ostream)
  "Serializes a message object of type '<ImuOffsetService-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'value))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ImuOffsetService-response>) istream)
  "Deserializes a message object of type '<ImuOffsetService-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ImuOffsetService-response>)))
  "Returns string type for a service object of type '<ImuOffsetService-response>"
  "setting_msg/ImuOffsetServiceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ImuOffsetService-response)))
  "Returns string type for a service object of type 'ImuOffsetService-response"
  "setting_msg/ImuOffsetServiceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ImuOffsetService-response>)))
  "Returns md5sum for a message object of type '<ImuOffsetService-response>"
  "f17e28b74ba2da00db559b27e3cb5a7f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ImuOffsetService-response)))
  "Returns md5sum for a message object of type 'ImuOffsetService-response"
  "f17e28b74ba2da00db559b27e3cb5a7f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ImuOffsetService-response>)))
  "Returns full string definition for message of type '<ImuOffsetService-response>"
  (cl:format cl:nil "string value~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ImuOffsetService-response)))
  "Returns full string definition for message of type 'ImuOffsetService-response"
  (cl:format cl:nil "string value~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ImuOffsetService-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'value))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ImuOffsetService-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ImuOffsetService-response
    (cl:cons ':value (value msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ImuOffsetService)))
  'ImuOffsetService-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ImuOffsetService)))
  'ImuOffsetService-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ImuOffsetService)))
  "Returns string type for a service object of type '<ImuOffsetService>"
  "setting_msg/ImuOffsetService")