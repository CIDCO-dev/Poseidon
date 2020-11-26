;; Auto-generated. Do not edit!


(when (boundp 'raspberrypi_vitals_msg::sysinfo)
  (if (not (find-package "RASPBERRYPI_VITALS_MSG"))
    (make-package "RASPBERRYPI_VITALS_MSG"))
  (shadow 'sysinfo (find-package "RASPBERRYPI_VITALS_MSG")))
(unless (find-package "RASPBERRYPI_VITALS_MSG::SYSINFO")
  (make-package "RASPBERRYPI_VITALS_MSG::SYSINFO"))

(in-package "ROS")
;;//! \htmlinclude sysinfo.msg.html


(defclass raspberrypi_vitals_msg::sysinfo
  :super ros::object
  :slots (_header _cputemp _cpuload _freeram _freehdd _uptime _vbat _rh _temp _psi ))

(defmethod raspberrypi_vitals_msg::sysinfo
  (:init
   (&key
    ((:header __header) 0.0)
    ((:cputemp __cputemp) 0.0)
    ((:cpuload __cpuload) 0.0)
    ((:freeram __freeram) 0.0)
    ((:freehdd __freehdd) 0.0)
    ((:uptime __uptime) 0.0)
    ((:vbat __vbat) 0.0)
    ((:rh __rh) 0.0)
    ((:temp __temp) 0.0)
    ((:psi __psi) 0.0)
    )
   (send-super :init)
   (setq _header (float __header))
   (setq _cputemp (float __cputemp))
   (setq _cpuload (float __cpuload))
   (setq _freeram (float __freeram))
   (setq _freehdd (float __freehdd))
   (setq _uptime (float __uptime))
   (setq _vbat (float __vbat))
   (setq _rh (float __rh))
   (setq _temp (float __temp))
   (setq _psi (float __psi))
   self)
  (:header
   (&optional __header)
   (if __header (setq _header __header)) _header)
  (:cputemp
   (&optional __cputemp)
   (if __cputemp (setq _cputemp __cputemp)) _cputemp)
  (:cpuload
   (&optional __cpuload)
   (if __cpuload (setq _cpuload __cpuload)) _cpuload)
  (:freeram
   (&optional __freeram)
   (if __freeram (setq _freeram __freeram)) _freeram)
  (:freehdd
   (&optional __freehdd)
   (if __freehdd (setq _freehdd __freehdd)) _freehdd)
  (:uptime
   (&optional __uptime)
   (if __uptime (setq _uptime __uptime)) _uptime)
  (:vbat
   (&optional __vbat)
   (if __vbat (setq _vbat __vbat)) _vbat)
  (:rh
   (&optional __rh)
   (if __rh (setq _rh __rh)) _rh)
  (:temp
   (&optional __temp)
   (if __temp (setq _temp __temp)) _temp)
  (:psi
   (&optional __psi)
   (if __psi (setq _psi __psi)) _psi)
  (:serialization-length
   ()
   (+
    ;; float64 _header
    8
    ;; float64 _cputemp
    8
    ;; float64 _cpuload
    8
    ;; float64 _freeram
    8
    ;; float64 _freehdd
    8
    ;; float64 _uptime
    8
    ;; float64 _vbat
    8
    ;; float64 _rh
    8
    ;; float64 _temp
    8
    ;; float64 _psi
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _header
       (sys::poke _header (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _cputemp
       (sys::poke _cputemp (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _cpuload
       (sys::poke _cpuload (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _freeram
       (sys::poke _freeram (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _freehdd
       (sys::poke _freehdd (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _uptime
       (sys::poke _uptime (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _vbat
       (sys::poke _vbat (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _rh
       (sys::poke _rh (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _temp
       (sys::poke _temp (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _psi
       (sys::poke _psi (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _header
     (setq _header (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _cputemp
     (setq _cputemp (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _cpuload
     (setq _cpuload (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _freeram
     (setq _freeram (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _freehdd
     (setq _freehdd (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _uptime
     (setq _uptime (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _vbat
     (setq _vbat (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _rh
     (setq _rh (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _temp
     (setq _temp (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _psi
     (setq _psi (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get raspberrypi_vitals_msg::sysinfo :md5sum-) "edf8691ad7d4ea111d25d08805390e3b")
(setf (get raspberrypi_vitals_msg::sysinfo :datatype-) "raspberrypi_vitals_msg/sysinfo")
(setf (get raspberrypi_vitals_msg::sysinfo :definition-)
      "float64 header
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



(provide :raspberrypi_vitals_msg/sysinfo "edf8691ad7d4ea111d25d08805390e3b")


