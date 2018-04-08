;; Auto-generated. Do not edit!


(when (boundp 'cooperative_driving_networking::Platooning)
  (if (not (find-package "COOPERATIVE_DRIVING_NETWORKING"))
    (make-package "COOPERATIVE_DRIVING_NETWORKING"))
  (shadow 'Platooning (find-package "COOPERATIVE_DRIVING_NETWORKING")))
(unless (find-package "COOPERATIVE_DRIVING_NETWORKING::PLATOONING")
  (make-package "COOPERATIVE_DRIVING_NETWORKING::PLATOONING"))

(in-package "ROS")
;;//! \htmlinclude Platooning.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass cooperative_driving_networking::Platooning
  :super ros::object
  :slots (_header _message_id _sender_id _platoon_id _leader_id _robot_role _predecessor_id _maneuver ))

(defmethod cooperative_driving_networking::Platooning
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:message_id __message_id) 0)
    ((:sender_id __sender_id) 0)
    ((:platoon_id __platoon_id) 0)
    ((:leader_id __leader_id) 0)
    ((:robot_role __robot_role) "")
    ((:predecessor_id __predecessor_id) 0)
    ((:maneuver __maneuver) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _message_id (round __message_id))
   (setq _sender_id (round __sender_id))
   (setq _platoon_id (round __platoon_id))
   (setq _leader_id (round __leader_id))
   (setq _robot_role (string __robot_role))
   (setq _predecessor_id (round __predecessor_id))
   (setq _maneuver (string __maneuver))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:message_id
   (&optional __message_id)
   (if __message_id (setq _message_id __message_id)) _message_id)
  (:sender_id
   (&optional __sender_id)
   (if __sender_id (setq _sender_id __sender_id)) _sender_id)
  (:platoon_id
   (&optional __platoon_id)
   (if __platoon_id (setq _platoon_id __platoon_id)) _platoon_id)
  (:leader_id
   (&optional __leader_id)
   (if __leader_id (setq _leader_id __leader_id)) _leader_id)
  (:robot_role
   (&optional __robot_role)
   (if __robot_role (setq _robot_role __robot_role)) _robot_role)
  (:predecessor_id
   (&optional __predecessor_id)
   (if __predecessor_id (setq _predecessor_id __predecessor_id)) _predecessor_id)
  (:maneuver
   (&optional __maneuver)
   (if __maneuver (setq _maneuver __maneuver)) _maneuver)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint32 _message_id
    4
    ;; uint8 _sender_id
    1
    ;; uint8 _platoon_id
    1
    ;; uint8 _leader_id
    1
    ;; string _robot_role
    4 (length _robot_role)
    ;; uint8 _predecessor_id
    1
    ;; string _maneuver
    4 (length _maneuver)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint32 _message_id
       (write-long _message_id s)
     ;; uint8 _sender_id
       (write-byte _sender_id s)
     ;; uint8 _platoon_id
       (write-byte _platoon_id s)
     ;; uint8 _leader_id
       (write-byte _leader_id s)
     ;; string _robot_role
       (write-long (length _robot_role) s) (princ _robot_role s)
     ;; uint8 _predecessor_id
       (write-byte _predecessor_id s)
     ;; string _maneuver
       (write-long (length _maneuver) s) (princ _maneuver s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint32 _message_id
     (setq _message_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint8 _sender_id
     (setq _sender_id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _platoon_id
     (setq _platoon_id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _leader_id
     (setq _leader_id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; string _robot_role
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _robot_role (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint8 _predecessor_id
     (setq _predecessor_id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; string _maneuver
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _maneuver (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get cooperative_driving_networking::Platooning :md5sum-) "ab1ba68fe20a61da2fb08021e8187972")
(setf (get cooperative_driving_networking::Platooning :datatype-) "cooperative_driving_networking/Platooning")
(setf (get cooperative_driving_networking::Platooning :definition-)
      "# Used in the demo application for the token passing protocol

# Common message header
# Standard ROS message header
Header header
# Applicationwise uniquie identifier of this message content
uint32 message_id
# The id of the robot which sends this message
uint8 sender_id
# Specific data of this message type
# TODO
uint8 platoon_id
# TODO
uint8 leader_id
# TODO
string robot_role
# TODO
uint8 predecessor_id
# TODO
string maneuver

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



(provide :cooperative_driving_networking/Platooning "ab1ba68fe20a61da2fb08021e8187972")


