;; Auto-generated. Do not edit!


(when (boundp 'cooperative_driving_networking::Token)
  (if (not (find-package "COOPERATIVE_DRIVING_NETWORKING"))
    (make-package "COOPERATIVE_DRIVING_NETWORKING"))
  (shadow 'Token (find-package "COOPERATIVE_DRIVING_NETWORKING")))
(unless (find-package "COOPERATIVE_DRIVING_NETWORKING::TOKEN")
  (make-package "COOPERATIVE_DRIVING_NETWORKING::TOKEN"))

(in-package "ROS")
;;//! \htmlinclude Token.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass cooperative_driving_networking::Token
  :super ros::object
  :slots (_header _message_id _sender_id _token_id ))

(defmethod cooperative_driving_networking::Token
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:message_id __message_id) 0)
    ((:sender_id __sender_id) 0)
    ((:token_id __token_id) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _message_id (round __message_id))
   (setq _sender_id (round __sender_id))
   (setq _token_id (round __token_id))
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
  (:token_id
   (&optional __token_id)
   (if __token_id (setq _token_id __token_id)) _token_id)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint32 _message_id
    4
    ;; uint8 _sender_id
    1
    ;; uint32 _token_id
    4
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
     ;; uint32 _token_id
       (write-long _token_id s)
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
   ;; uint32 _token_id
     (setq _token_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get cooperative_driving_networking::Token :md5sum-) "2cf72de3eab0cac539f4018d2c17b401")
(setf (get cooperative_driving_networking::Token :datatype-) "cooperative_driving_networking/Token")
(setf (get cooperative_driving_networking::Token :definition-)
      "# Used in the demo application for the token passing protocol

# Common message header
# Standard ROS message header
Header header
# Applicationwise uniquie identifier of this message content
uint32 message_id
# The id of the robot which sends this message
uint8 sender_id
# Specific data of this message type
# The token forwared by this messages represented via its id
uint32 token_id

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



(provide :cooperative_driving_networking/Token "2cf72de3eab0cac539f4018d2c17b401")


