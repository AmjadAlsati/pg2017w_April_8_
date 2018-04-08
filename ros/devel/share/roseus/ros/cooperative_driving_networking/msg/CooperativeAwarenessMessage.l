;; Auto-generated. Do not edit!


(when (boundp 'cooperative_driving_networking::CooperativeAwarenessMessage)
  (if (not (find-package "COOPERATIVE_DRIVING_NETWORKING"))
    (make-package "COOPERATIVE_DRIVING_NETWORKING"))
  (shadow 'CooperativeAwarenessMessage (find-package "COOPERATIVE_DRIVING_NETWORKING")))
(unless (find-package "COOPERATIVE_DRIVING_NETWORKING::COOPERATIVEAWARENESSMESSAGE")
  (make-package "COOPERATIVE_DRIVING_NETWORKING::COOPERATIVEAWARENESSMESSAGE"))

(in-package "ROS")
;;//! \htmlinclude CooperativeAwarenessMessage.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass cooperative_driving_networking::CooperativeAwarenessMessage
  :super ros::object
  :slots (_header _message_id _sender_id _velocity _steering _posx _posy _posz _road_id _lane_id ))

(defmethod cooperative_driving_networking::CooperativeAwarenessMessage
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:message_id __message_id) 0)
    ((:sender_id __sender_id) 0)
    ((:velocity __velocity) 0.0)
    ((:steering __steering) 0.0)
    ((:posx __posx) 0.0)
    ((:posy __posy) 0.0)
    ((:posz __posz) 0.0)
    ((:road_id __road_id) 0)
    ((:lane_id __lane_id) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _message_id (round __message_id))
   (setq _sender_id (round __sender_id))
   (setq _velocity (float __velocity))
   (setq _steering (float __steering))
   (setq _posx (float __posx))
   (setq _posy (float __posy))
   (setq _posz (float __posz))
   (setq _road_id (round __road_id))
   (setq _lane_id (round __lane_id))
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
  (:velocity
   (&optional __velocity)
   (if __velocity (setq _velocity __velocity)) _velocity)
  (:steering
   (&optional __steering)
   (if __steering (setq _steering __steering)) _steering)
  (:posx
   (&optional __posx)
   (if __posx (setq _posx __posx)) _posx)
  (:posy
   (&optional __posy)
   (if __posy (setq _posy __posy)) _posy)
  (:posz
   (&optional __posz)
   (if __posz (setq _posz __posz)) _posz)
  (:road_id
   (&optional __road_id)
   (if __road_id (setq _road_id __road_id)) _road_id)
  (:lane_id
   (&optional __lane_id)
   (if __lane_id (setq _lane_id __lane_id)) _lane_id)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint32 _message_id
    4
    ;; uint8 _sender_id
    1
    ;; float32 _velocity
    4
    ;; float32 _steering
    4
    ;; float32 _posx
    4
    ;; float32 _posy
    4
    ;; float32 _posz
    4
    ;; uint8 _road_id
    1
    ;; uint8 _lane_id
    1
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
     ;; float32 _velocity
       (sys::poke _velocity (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _steering
       (sys::poke _steering (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _posx
       (sys::poke _posx (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _posy
       (sys::poke _posy (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _posz
       (sys::poke _posz (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint8 _road_id
       (write-byte _road_id s)
     ;; uint8 _lane_id
       (write-byte _lane_id s)
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
   ;; float32 _velocity
     (setq _velocity (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _steering
     (setq _steering (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _posx
     (setq _posx (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _posy
     (setq _posy (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _posz
     (setq _posz (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint8 _road_id
     (setq _road_id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _lane_id
     (setq _lane_id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get cooperative_driving_networking::CooperativeAwarenessMessage :md5sum-) "55c0c4a0b38cbbc32116f929677739b2")
(setf (get cooperative_driving_networking::CooperativeAwarenessMessage :datatype-) "cooperative_driving_networking/CooperativeAwarenessMessage")
(setf (get cooperative_driving_networking::CooperativeAwarenessMessage :definition-)
      "# Used in the application base class to represent a simple ETSI like CAMs

# Common message header
# Standard ROS message header
Header header
# Applicationwise uniquie identifier of this message content
uint32 message_id
# The id of the robot which sends this message
uint8 sender_id
# Specific data of this message type
# The current velocity value of this robot
float32 velocity
# The current steering value of this robot
float32 steering
# The current x coordinate of this robot
float32 posx
# The current y coordinate of this robot
float32 posy
# The current z coordinate of this robot
float32 posz
# The id of the road this robot is currently driving on
uint8 road_id
# The id of the lane this robot is currently driving on
uint8 lane_id

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



(provide :cooperative_driving_networking/CooperativeAwarenessMessage "55c0c4a0b38cbbc32116f929677739b2")


