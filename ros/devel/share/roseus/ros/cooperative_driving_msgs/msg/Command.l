;; Auto-generated. Do not edit!


(when (boundp 'cooperative_driving_msgs::Command)
  (if (not (find-package "COOPERATIVE_DRIVING_MSGS"))
    (make-package "COOPERATIVE_DRIVING_MSGS"))
  (shadow 'Command (find-package "COOPERATIVE_DRIVING_MSGS")))
(unless (find-package "COOPERATIVE_DRIVING_MSGS::COMMAND")
  (make-package "COOPERATIVE_DRIVING_MSGS::COMMAND"))

(in-package "ROS")
;;//! \htmlinclude Command.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass cooperative_driving_msgs::Command
  :super ros::object
  :slots (_header _message_id _sender_id _command _turn ))

(defmethod cooperative_driving_msgs::Command
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:message_id __message_id) 0)
    ((:sender_id __sender_id) 0)
    ((:command __command) "")
    ((:turn __turn) (instance cooperative_driving_msgs::Direction :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _message_id (round __message_id))
   (setq _sender_id (round __sender_id))
   (setq _command (string __command))
   (setq _turn __turn)
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
  (:command
   (&optional __command)
   (if __command (setq _command __command)) _command)
  (:turn
   (&rest __turn)
   (if (keywordp (car __turn))
       (send* _turn __turn)
     (progn
       (if __turn (setq _turn (car __turn)))
       _turn)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint32 _message_id
    4
    ;; uint8 _sender_id
    1
    ;; string _command
    4 (length _command)
    ;; cooperative_driving_msgs/Direction _turn
    (send _turn :serialization-length)
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
     ;; string _command
       (write-long (length _command) s) (princ _command s)
     ;; cooperative_driving_msgs/Direction _turn
       (send _turn :serialize s)
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
   ;; string _command
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _command (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; cooperative_driving_msgs/Direction _turn
     (send _turn :deserialize buf ptr-) (incf ptr- (send _turn :serialization-length))
   ;;
   self)
  )

(setf (get cooperative_driving_msgs::Command :md5sum-) "a2cc60514e9f00b5f108e5118c24aaa9")
(setf (get cooperative_driving_msgs::Command :datatype-) "cooperative_driving_msgs/Command")
(setf (get cooperative_driving_msgs::Command :definition-)
      "## Command a direction change at an upcoming crossing
# Common message header
# Standard ROS message header
Header header
# Define the direction to take at the upcoming crossing
uint32 message_id
# The id of the robot which sends this message
uint8 sender_id
# Specific data of this message type

# Commnand to be executed
string command
# Parameters for the command

# Parameter for turn command
Direction turn

#TODO: Add other command parameters here
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
MSG: cooperative_driving_msgs/Direction
## Define a direction of a detected crossing, i.e., a turn that can possibly be taken

# Possible turn at a crossing ('straight', 'left', or 'right')
string direction
")



(provide :cooperative_driving_msgs/Command "a2cc60514e9f00b5f108e5118c24aaa9")


