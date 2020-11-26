; Auto-generated. Do not edit!


(cl:in-package raspberrypi_vitals_msg-msg)


;//! \htmlinclude sysinfo.msg.html

(cl:defclass <sysinfo> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type cl:float
    :initform 0.0)
   (cputemp
    :reader cputemp
    :initarg :cputemp
    :type cl:float
    :initform 0.0)
   (cpuload
    :reader cpuload
    :initarg :cpuload
    :type cl:float
    :initform 0.0)
   (freeram
    :reader freeram
    :initarg :freeram
    :type cl:float
    :initform 0.0)
   (freehdd
    :reader freehdd
    :initarg :freehdd
    :type cl:float
    :initform 0.0)
   (uptime
    :reader uptime
    :initarg :uptime
    :type cl:float
    :initform 0.0)
   (vbat
    :reader vbat
    :initarg :vbat
    :type cl:float
    :initform 0.0)
   (rh
    :reader rh
    :initarg :rh
    :type cl:float
    :initform 0.0)
   (temp
    :reader temp
    :initarg :temp
    :type cl:float
    :initform 0.0)
   (psi
    :reader psi
    :initarg :psi
    :type cl:float
    :initform 0.0))
)

(cl:defclass sysinfo (<sysinfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <sysinfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'sysinfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name raspberrypi_vitals_msg-msg:<sysinfo> is deprecated: use raspberrypi_vitals_msg-msg:sysinfo instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <sysinfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader raspberrypi_vitals_msg-msg:header-val is deprecated.  Use raspberrypi_vitals_msg-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'cputemp-val :lambda-list '(m))
(cl:defmethod cputemp-val ((m <sysinfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader raspberrypi_vitals_msg-msg:cputemp-val is deprecated.  Use raspberrypi_vitals_msg-msg:cputemp instead.")
  (cputemp m))

(cl:ensure-generic-function 'cpuload-val :lambda-list '(m))
(cl:defmethod cpuload-val ((m <sysinfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader raspberrypi_vitals_msg-msg:cpuload-val is deprecated.  Use raspberrypi_vitals_msg-msg:cpuload instead.")
  (cpuload m))

(cl:ensure-generic-function 'freeram-val :lambda-list '(m))
(cl:defmethod freeram-val ((m <sysinfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader raspberrypi_vitals_msg-msg:freeram-val is deprecated.  Use raspberrypi_vitals_msg-msg:freeram instead.")
  (freeram m))

(cl:ensure-generic-function 'freehdd-val :lambda-list '(m))
(cl:defmethod freehdd-val ((m <sysinfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader raspberrypi_vitals_msg-msg:freehdd-val is deprecated.  Use raspberrypi_vitals_msg-msg:freehdd instead.")
  (freehdd m))

(cl:ensure-generic-function 'uptime-val :lambda-list '(m))
(cl:defmethod uptime-val ((m <sysinfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader raspberrypi_vitals_msg-msg:uptime-val is deprecated.  Use raspberrypi_vitals_msg-msg:uptime instead.")
  (uptime m))

(cl:ensure-generic-function 'vbat-val :lambda-list '(m))
(cl:defmethod vbat-val ((m <sysinfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader raspberrypi_vitals_msg-msg:vbat-val is deprecated.  Use raspberrypi_vitals_msg-msg:vbat instead.")
  (vbat m))

(cl:ensure-generic-function 'rh-val :lambda-list '(m))
(cl:defmethod rh-val ((m <sysinfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader raspberrypi_vitals_msg-msg:rh-val is deprecated.  Use raspberrypi_vitals_msg-msg:rh instead.")
  (rh m))

(cl:ensure-generic-function 'temp-val :lambda-list '(m))
(cl:defmethod temp-val ((m <sysinfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader raspberrypi_vitals_msg-msg:temp-val is deprecated.  Use raspberrypi_vitals_msg-msg:temp instead.")
  (temp m))

(cl:ensure-generic-function 'psi-val :lambda-list '(m))
(cl:defmethod psi-val ((m <sysinfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader raspberrypi_vitals_msg-msg:psi-val is deprecated.  Use raspberrypi_vitals_msg-msg:psi instead.")
  (psi m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <sysinfo>) ostream)
  "Serializes a message object of type '<sysinfo>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'header))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'cputemp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'cpuload))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'freeram))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'freehdd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'uptime))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'vbat))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'rh))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'temp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'psi))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <sysinfo>) istream)
  "Deserializes a message object of type '<sysinfo>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'header) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cputemp) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cpuload) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'freeram) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'freehdd) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'uptime) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vbat) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rh) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'temp) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'psi) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<sysinfo>)))
  "Returns string type for a message object of type '<sysinfo>"
  "raspberrypi_vitals_msg/sysinfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'sysinfo)))
  "Returns string type for a message object of type 'sysinfo"
  "raspberrypi_vitals_msg/sysinfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<sysinfo>)))
  "Returns md5sum for a message object of type '<sysinfo>"
  "edf8691ad7d4ea111d25d08805390e3b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'sysinfo)))
  "Returns md5sum for a message object of type 'sysinfo"
  "edf8691ad7d4ea111d25d08805390e3b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<sysinfo>)))
  "Returns full string definition for message of type '<sysinfo>"
  (cl:format cl:nil "float64 header~%float64 cputemp~%float64 cpuload~%float64 freeram~%float64 freehdd~%float64 uptime~%float64 vbat~%float64 rh~%float64 temp~%float64 psi~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'sysinfo)))
  "Returns full string definition for message of type 'sysinfo"
  (cl:format cl:nil "float64 header~%float64 cputemp~%float64 cpuload~%float64 freeram~%float64 freehdd~%float64 uptime~%float64 vbat~%float64 rh~%float64 temp~%float64 psi~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <sysinfo>))
  (cl:+ 0
     8
     8
     8
     8
     8
     8
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <sysinfo>))
  "Converts a ROS message object to a list"
  (cl:list 'sysinfo
    (cl:cons ':header (header msg))
    (cl:cons ':cputemp (cputemp msg))
    (cl:cons ':cpuload (cpuload msg))
    (cl:cons ':freeram (freeram msg))
    (cl:cons ':freehdd (freehdd msg))
    (cl:cons ':uptime (uptime msg))
    (cl:cons ':vbat (vbat msg))
    (cl:cons ':rh (rh msg))
    (cl:cons ':temp (temp msg))
    (cl:cons ':psi (psi msg))
))
