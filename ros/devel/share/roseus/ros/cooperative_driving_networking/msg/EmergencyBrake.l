;; Auto-generated. Do not edit!


(when (boundp 'cooperative_driving_networking::EmergencyBrake)
  (if (not (find-package "COOPERATIVE_DRIVING_NETWORKING"))
    (make-package "COOPERATIVE_DRIVING_NETWORKING"))
  (shadow 'EmergencyBrake (find-package "COOPERATIVE_DRIVING_NETWORKING")))
(unless (find-package "COOPERATIVE_DRIVING_NETWORKING::EMERGENCYBRAKE")
  (make-package "COOPERATIVE_DRIVING_NETWORKING::EMERGENCYBRAKE"))

(in-package "ROS")
;;//! \htmlinclude EmergencyBrake.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass cooperative_driving_networking::EmergencyBrake
  :super ros::object
  :slots (_header _sender_id _enable ))

(defmethod cooperative_driving_networking::EmergencyBrake
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:sender_id __sender_id) 0)
    ((:enable __enable) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _sender_id (round __sender_id))
   (setq _enable __enable)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:sender_id
   (&optional __sender_id)
   (if __sender_id (setq _sender_id __sender_id)) _sender_id)
  (:enable
   (&optional __enable)
   (if __enable (setq _enable __enable)) _enable)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _sender_id
    1
    ;; bool _enable
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _sender_id
       (write-byte _sender_id s)
     ;; bool _enable
       (if _enable (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _sender_id
     (setq _sender_id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _enable
     (setq _enable (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get cooperative_driving_networking::EmergencyBrake :md5sum-) "9f70b5ab2cb5f15d7d35f30d081697ad")
(setf (get cooperative_driving_networking::EmergencyBrake :datatype-) "cooperative_driving_networking/EmergencyBrake")
(setf (get cooperative_driving_networking::EmergencyBrake :definition-)
      "# Used in the demo application for the token passing protocol

# Common message header
# Standard ROS message header
Header header
# The id of the robot which sends this message
uint8 sender_id
# Specific data of this message type
# Flag to indicate whether an emergency braking situation occured or is resolved
bool enable

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

")



(provide :cooperative_driving_networking/EmergencyBrake "9f70b5ab2cb5f15d7d35f30d081697ad")


