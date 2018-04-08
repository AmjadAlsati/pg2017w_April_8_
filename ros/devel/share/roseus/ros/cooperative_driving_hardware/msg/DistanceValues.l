;; Auto-generated. Do not edit!


(when (boundp 'cooperative_driving_hardware::DistanceValues)
  (if (not (find-package "COOPERATIVE_DRIVING_HARDWARE"))
    (make-package "COOPERATIVE_DRIVING_HARDWARE"))
  (shadow 'DistanceValues (find-package "COOPERATIVE_DRIVING_HARDWARE")))
(unless (find-package "COOPERATIVE_DRIVING_HARDWARE::DISTANCEVALUES")
  (make-package "COOPERATIVE_DRIVING_HARDWARE::DISTANCEVALUES"))

(in-package "ROS")
;;//! \htmlinclude DistanceValues.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass cooperative_driving_hardware::DistanceValues
  :super ros::object
  :slots (_header _values ))

(defmethod cooperative_driving_hardware::DistanceValues
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:values __values) (let (r) (dotimes (i 0) (push (instance cooperative_driving_hardware::DistanceValue :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _values __values)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:values
   (&rest __values)
   (if (keywordp (car __values))
       (send* _values __values)
     (progn
       (if __values (setq _values (car __values)))
       _values)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; cooperative_driving_hardware/DistanceValue[] _values
    (apply #'+ (send-all _values :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; cooperative_driving_hardware/DistanceValue[] _values
     (write-long (length _values) s)
     (dolist (elem _values)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; cooperative_driving_hardware/DistanceValue[] _values
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _values (let (r) (dotimes (i n) (push (instance cooperative_driving_hardware::DistanceValue :init) r)) r))
     (dolist (elem- _values)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get cooperative_driving_hardware::DistanceValues :md5sum-) "cdf6872394aabe6065b7a71d82668d8b")
(setf (get cooperative_driving_hardware::DistanceValues :datatype-) "cooperative_driving_hardware/DistanceValues")
(setf (get cooperative_driving_hardware::DistanceValues :definition-)
      "# Used for distributing distance sensor readings

# Standard ROS message header
Header header
# An array of DistanceValue messages represent include the sensor readings
DistanceValue[] values

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
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: cooperative_driving_hardware/DistanceValue
# Custom message type represent a reading from a distance sensor used for reading

# The frame_id (name) of this sensor
string frame_id
# The reading of this sensor
# Values in m
float32 distance

")



(provide :cooperative_driving_hardware/DistanceValues "cdf6872394aabe6065b7a71d82668d8b")


