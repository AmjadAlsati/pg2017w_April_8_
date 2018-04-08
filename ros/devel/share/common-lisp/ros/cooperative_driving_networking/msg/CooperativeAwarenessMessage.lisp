; Auto-generated. Do not edit!


(cl:in-package cooperative_driving_networking-msg)


;//! \htmlinclude CooperativeAwarenessMessage.msg.html

(cl:defclass <CooperativeAwarenessMessage> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (message_id
    :reader message_id
    :initarg :message_id
    :type cl:integer
    :initform 0)
   (sender_id
    :reader sender_id
    :initarg :sender_id
    :type cl:fixnum
    :initform 0)
   (velocity
    :reader velocity
    :initarg :velocity
    :type cl:float
    :initform 0.0)
   (steering
    :reader steering
    :initarg :steering
    :type cl:float
    :initform 0.0)
   (posx
    :reader posx
    :initarg :posx
    :type cl:float
    :initform 0.0)
   (posy
    :reader posy
    :initarg :posy
    :type cl:float
    :initform 0.0)
   (posz
    :reader posz
    :initarg :posz
    :type cl:float
    :initform 0.0)
   (road_id
    :reader road_id
    :initarg :road_id
    :type cl:fixnum
    :initform 0)
   (lane_id
    :reader lane_id
    :initarg :lane_id
    :type cl:fixnum
    :initform 0))
)

(cl:defclass CooperativeAwarenessMessage (<CooperativeAwarenessMessage>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CooperativeAwarenessMessage>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CooperativeAwarenessMessage)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cooperative_driving_networking-msg:<CooperativeAwarenessMessage> is deprecated: use cooperative_driving_networking-msg:CooperativeAwarenessMessage instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <CooperativeAwarenessMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_networking-msg:header-val is deprecated.  Use cooperative_driving_networking-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'message_id-val :lambda-list '(m))
(cl:defmethod message_id-val ((m <CooperativeAwarenessMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_networking-msg:message_id-val is deprecated.  Use cooperative_driving_networking-msg:message_id instead.")
  (message_id m))

(cl:ensure-generic-function 'sender_id-val :lambda-list '(m))
(cl:defmethod sender_id-val ((m <CooperativeAwarenessMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_networking-msg:sender_id-val is deprecated.  Use cooperative_driving_networking-msg:sender_id instead.")
  (sender_id m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <CooperativeAwarenessMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_networking-msg:velocity-val is deprecated.  Use cooperative_driving_networking-msg:velocity instead.")
  (velocity m))

(cl:ensure-generic-function 'steering-val :lambda-list '(m))
(cl:defmethod steering-val ((m <CooperativeAwarenessMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_networking-msg:steering-val is deprecated.  Use cooperative_driving_networking-msg:steering instead.")
  (steering m))

(cl:ensure-generic-function 'posx-val :lambda-list '(m))
(cl:defmethod posx-val ((m <CooperativeAwarenessMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_networking-msg:posx-val is deprecated.  Use cooperative_driving_networking-msg:posx instead.")
  (posx m))

(cl:ensure-generic-function 'posy-val :lambda-list '(m))
(cl:defmethod posy-val ((m <CooperativeAwarenessMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_networking-msg:posy-val is deprecated.  Use cooperative_driving_networking-msg:posy instead.")
  (posy m))

(cl:ensure-generic-function 'posz-val :lambda-list '(m))
(cl:defmethod posz-val ((m <CooperativeAwarenessMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_networking-msg:posz-val is deprecated.  Use cooperative_driving_networking-msg:posz instead.")
  (posz m))

(cl:ensure-generic-function 'road_id-val :lambda-list '(m))
(cl:defmethod road_id-val ((m <CooperativeAwarenessMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_networking-msg:road_id-val is deprecated.  Use cooperative_driving_networking-msg:road_id instead.")
  (road_id m))

(cl:ensure-generic-function 'lane_id-val :lambda-list '(m))
(cl:defmethod lane_id-val ((m <CooperativeAwarenessMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_networking-msg:lane_id-val is deprecated.  Use cooperative_driving_networking-msg:lane_id instead.")
  (lane_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CooperativeAwarenessMessage>) ostream)
  "Serializes a message object of type '<CooperativeAwarenessMessage>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'message_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'message_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'message_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'message_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sender_id)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'steering))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'posx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'posy))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'posz))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'road_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'lane_id)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CooperativeAwarenessMessage>) istream)
  "Deserializes a message object of type '<CooperativeAwarenessMessage>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'message_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'message_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'message_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'message_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sender_id)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'velocity) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'steering) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'posx) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'posy) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'posz) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'road_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'lane_id)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CooperativeAwarenessMessage>)))
  "Returns string type for a message object of type '<CooperativeAwarenessMessage>"
  "cooperative_driving_networking/CooperativeAwarenessMessage")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CooperativeAwarenessMessage)))
  "Returns string type for a message object of type 'CooperativeAwarenessMessage"
  "cooperative_driving_networking/CooperativeAwarenessMessage")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CooperativeAwarenessMessage>)))
  "Returns md5sum for a message object of type '<CooperativeAwarenessMessage>"
  "55c0c4a0b38cbbc32116f929677739b2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CooperativeAwarenessMessage)))
  "Returns md5sum for a message object of type 'CooperativeAwarenessMessage"
  "55c0c4a0b38cbbc32116f929677739b2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CooperativeAwarenessMessage>)))
  "Returns full string definition for message of type '<CooperativeAwarenessMessage>"
  (cl:format cl:nil "# Used in the application base class to represent a simple ETSI like CAMs~%~%# Common message header~%# Standard ROS message header~%Header header~%# Applicationwise uniquie identifier of this message content~%uint32 message_id~%# The id of the robot which sends this message~%uint8 sender_id~%# Specific data of this message type~%# The current velocity value of this robot~%float32 velocity~%# The current steering value of this robot~%float32 steering~%# The current x coordinate of this robot~%float32 posx~%# The current y coordinate of this robot~%float32 posy~%# The current z coordinate of this robot~%float32 posz~%# The id of the road this robot is currently driving on~%uint8 road_id~%# The id of the lane this robot is currently driving on~%uint8 lane_id~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CooperativeAwarenessMessage)))
  "Returns full string definition for message of type 'CooperativeAwarenessMessage"
  (cl:format cl:nil "# Used in the application base class to represent a simple ETSI like CAMs~%~%# Common message header~%# Standard ROS message header~%Header header~%# Applicationwise uniquie identifier of this message content~%uint32 message_id~%# The id of the robot which sends this message~%uint8 sender_id~%# Specific data of this message type~%# The current velocity value of this robot~%float32 velocity~%# The current steering value of this robot~%float32 steering~%# The current x coordinate of this robot~%float32 posx~%# The current y coordinate of this robot~%float32 posy~%# The current z coordinate of this robot~%float32 posz~%# The id of the road this robot is currently driving on~%uint8 road_id~%# The id of the lane this robot is currently driving on~%uint8 lane_id~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CooperativeAwarenessMessage>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     1
     4
     4
     4
     4
     4
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CooperativeAwarenessMessage>))
  "Converts a ROS message object to a list"
  (cl:list 'CooperativeAwarenessMessage
    (cl:cons ':header (header msg))
    (cl:cons ':message_id (message_id msg))
    (cl:cons ':sender_id (sender_id msg))
    (cl:cons ':velocity (velocity msg))
    (cl:cons ':steering (steering msg))
    (cl:cons ':posx (posx msg))
    (cl:cons ':posy (posy msg))
    (cl:cons ':posz (posz msg))
    (cl:cons ':road_id (road_id msg))
    (cl:cons ':lane_id (lane_id msg))
))
