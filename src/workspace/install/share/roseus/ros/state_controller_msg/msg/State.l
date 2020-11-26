;; Auto-generated. Do not edit!


(when (boundp 'state_controller_msg::State)
  (if (not (find-package "STATE_CONTROLLER_MSG"))
    (make-package "STATE_CONTROLLER_MSG"))
  (shadow 'State (find-package "STATE_CONTROLLER_MSG")))
(unless (find-package "STATE_CONTROLLER_MSG::STATE")
  (make-package "STATE_CONTROLLER_MSG::STATE"))

(in-package "ROS")
;;//! \htmlinclude State.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "RASPBERRYPI_VITALS_MSG"))
  (ros::roseus-add-msgs "raspberrypi_vitals_msg"))
(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))


(defclass state_controller_msg::State
  :super ros::object
  :slots (_position _imu _depth _vitals ))

(defmethod state_controller_msg::State
  (:init
   (&key
    ((:position __position) (instance sensor_msgs::NavSatFix :init))
    ((:imu __imu) (instance sensor_msgs::Imu :init))
    ((:depth __depth) (instance geometry_msgs::PointStamped :init))
    ((:vitals __vitals) (instance raspberrypi_vitals_msg::sysinfo :init))
    )
   (send-super :init)
   (setq _position __position)
   (setq _imu __imu)
   (setq _depth __depth)
   (setq _vitals __vitals)
   self)
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:imu
   (&rest __imu)
   (if (keywordp (car __imu))
       (send* _imu __imu)
     (progn
       (if __imu (setq _imu (car __imu)))
       _imu)))
  (:depth
   (&rest __depth)
   (if (keywordp (car __depth))
       (send* _depth __depth)
     (progn
       (if __depth (setq _depth (car __depth)))
       _depth)))
  (:vitals
   (&rest __vitals)
   (if (keywordp (car __vitals))
       (send* _vitals __vitals)
     (progn
       (if __vitals (setq _vitals (car __vitals)))
       _vitals)))
  (:serialization-length
   ()
   (+
    ;; sensor_msgs/NavSatFix _position
    (send _position :serialization-length)
    ;; sensor_msgs/Imu _imu
    (send _imu :serialization-length)
    ;; geometry_msgs/PointStamped _depth
    (send _depth :serialization-length)
    ;; raspberrypi_vitals_msg/sysinfo _vitals
    (send _vitals :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; sensor_msgs/NavSatFix _position
       (send _position :serialize s)
     ;; sensor_msgs/Imu _imu
       (send _imu :serialize s)
     ;; geometry_msgs/PointStamped _depth
       (send _depth :serialize s)
     ;; raspberrypi_vitals_msg/sysinfo _vitals
       (send _vitals :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; sensor_msgs/NavSatFix _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; sensor_msgs/Imu _imu
     (send _imu :deserialize buf ptr-) (incf ptr- (send _imu :serialization-length))
   ;; geometry_msgs/PointStamped _depth
     (send _depth :deserialize buf ptr-) (incf ptr- (send _depth :serialization-length))
   ;; raspberrypi_vitals_msg/sysinfo _vitals
     (send _vitals :deserialize buf ptr-) (incf ptr- (send _vitals :serialization-length))
   ;;
   self)
  )

(setf (get state_controller_msg::State :md5sum-) "e545755a8a0293f7a8344942fb13f11b")
(setf (get state_controller_msg::State :datatype-) "state_controller_msg/State")
(setf (get state_controller_msg::State :definition-)
      "sensor_msgs/NavSatFix position
sensor_msgs/Imu imu
geometry_msgs/PointStamped depth
raspberrypi_vitals_msg/sysinfo vitals

================================================================================
MSG: sensor_msgs/NavSatFix
# Navigation Satellite fix for any Global Navigation Satellite System
#
# Specified using the WGS 84 reference ellipsoid

# header.stamp specifies the ROS time for this measurement (the
#        corresponding satellite time may be reported using the
#        sensor_msgs/TimeReference message).
#
# header.frame_id is the frame of reference reported by the satellite
#        receiver, usually the location of the antenna.  This is a
#        Euclidean frame relative to the vehicle, not a reference
#        ellipsoid.
Header header

# satellite fix status information
NavSatStatus status

# Latitude [degrees]. Positive is north of equator; negative is south.
float64 latitude

# Longitude [degrees]. Positive is east of prime meridian; negative is west.
float64 longitude

# Altitude [m]. Positive is above the WGS 84 ellipsoid
# (quiet NaN if no altitude is available).
float64 altitude

# Position covariance [m^2] defined relative to a tangential plane
# through the reported position. The components are East, North, and
# Up (ENU), in row-major order.
#
# Beware: this coordinate system exhibits singularities at the poles.

float64[9] position_covariance

# If the covariance of the fix is known, fill it in completely. If the
# GPS receiver provides the variance of each measurement, put them
# along the diagonal. If only Dilution of Precision is available,
# estimate an approximate covariance from that.

uint8 COVARIANCE_TYPE_UNKNOWN = 0
uint8 COVARIANCE_TYPE_APPROXIMATED = 1
uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN = 2
uint8 COVARIANCE_TYPE_KNOWN = 3

uint8 position_covariance_type

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: sensor_msgs/NavSatStatus
# Navigation Satellite fix status for any Global Navigation Satellite System

# Whether to output an augmented fix is determined by both the fix
# type and the last time differential corrections were received.  A
# fix is valid when status >= STATUS_FIX.

int8 STATUS_NO_FIX =  -1        # unable to fix position
int8 STATUS_FIX =      0        # unaugmented fix
int8 STATUS_SBAS_FIX = 1        # with satellite-based augmentation
int8 STATUS_GBAS_FIX = 2        # with ground-based augmentation

int8 status

# Bits defining which Global Navigation Satellite System signals were
# used by the receiver.

uint16 SERVICE_GPS =     1
uint16 SERVICE_GLONASS = 2
uint16 SERVICE_COMPASS = 4      # includes BeiDou.
uint16 SERVICE_GALILEO = 8

uint16 service

================================================================================
MSG: sensor_msgs/Imu
# This is a message to hold data from an IMU (Inertial Measurement Unit)
#
# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
#
# If the covariance of the measurement is known, it should be filled in (if all you know is the 
# variance of each measurement, e.g. from the datasheet, just put those along the diagonal)
# A covariance matrix of all zeros will be interpreted as \"covariance unknown\", and to use the
# data a covariance will have to be assumed or gotten from some other source
#
# If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an orientation 
# estimate), please set element 0 of the associated covariance matrix to -1
# If you are interpreting this message, please check for a value of -1 in the first element of each 
# covariance matrix, and disregard the associated estimate.

Header header

geometry_msgs/Quaternion orientation
float64[9] orientation_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance # Row major x, y z 

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/PointStamped
# This represents a Point with reference coordinate frame and timestamp
Header header
Point point

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: raspberrypi_vitals_msg/sysinfo
float64 header
float64 cputemp
float64 cpuload
float64 freeram
float64 freehdd
float64 uptime
float64 vbat
float64 rh
float64 temp
float64 psi

")



(provide :state_controller_msg/State "e545755a8a0293f7a8344942fb13f11b")


