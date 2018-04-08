; Auto-generated. Do not edit!


(cl:in-package cooperative_driving_networking-msg)


;//! \htmlinclude Platooning.msg.html

(cl:defclass <Platooning> (roslisp-msg-protocol:ros-message)
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
   (platoon_id
    :reader platoon_id
    :initarg :platoon_id
    :type cl:fixnum
    :initform 0)
   (leader_id
    :reader leader_id
    :initarg :leader_id
    :type cl:fixnum
    :initform 0)
   (robot_role
    :reader robot_role
    :initarg :robot_role
    :type cl:string
    :initform "")
   (predecessor_id
    :reader predecessor_id
    :initarg :predecessor_id
    :type cl:fixnum
    :initform 0)
   (maneuver
    :reader maneuver
    :initarg :maneuver
    :type cl:string
    :initform ""))
)

(cl:defclass Platooning (<Platooning>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Platooning>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Platooning)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cooperative_driving_networking-msg:<Platooning> is deprecated: use cooperative_driving_networking-msg:Platooning instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Platooning>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_networking-msg:header-val is deprecated.  Use cooperative_driving_networking-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'message_id-val :lambda-list '(m))
(cl:defmethod message_id-val ((m <Platooning>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_networking-msg:message_id-val is deprecated.  Use cooperative_driving_networking-msg:message_id instead.")
  (message_id m))

(cl:ensure-generic-function 'sender_id-val :lambda-list '(m))
(cl:defmethod sender_id-val ((m <Platooning>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_networking-msg:sender_id-val is deprecated.  Use cooperative_driving_networking-msg:sender_id instead.")
  (sender_id m))

(cl:ensure-generic-function 'platoon_id-val :lambda-list '(m))
(cl:defmethod platoon_id-val ((m <Platooning>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_networking-msg:platoon_id-val is deprecated.  Use cooperative_driving_networking-msg:platoon_id instead.")
  (platoon_id m))

(cl:ensure-generic-function 'leader_id-val :lambda-list '(m))
(cl:defmethod leader_id-val ((m <Platooning>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_networking-msg:leader_id-val is deprecated.  Use cooperative_driving_networking-msg:leader_id instead.")
  (leader_id m))

(cl:ensure-generic-function 'robot_role-val :lambda-list '(m))
(cl:defmethod robot_role-val ((m <Platooning>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_networking-msg:robot_role-val is deprecated.  Use cooperative_driving_networking-msg:robot_role instead.")
  (robot_role m))

(cl:ensure-generic-function 'predecessor_id-val :lambda-list '(m))
(cl:defmethod predecessor_id-val ((m <Platooning>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_networking-msg:predecessor_id-val is deprecated.  Use cooperative_driving_networking-msg:predecessor_id instead.")
  (predecessor_id m))

(cl:ensure-generic-function 'maneuver-val :lambda-list '(m))
(cl:defmethod maneuver-val ((m <Platooning>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_networking-msg:maneuver-val is deprecated.  Use cooperative_driving_networking-msg:maneuver instead.")
  (maneuver m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Platooning>) ostream)
  "Serializes a message object of type '<Platooning>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'message_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'message_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'message_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'message_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sender_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'platoon_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'leader_id)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'robot_role))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'robot_role))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'predecessor_id)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'maneuver))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'maneuver))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Platooning>) istream)
  "Deserializes a message object of type '<Platooning>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'message_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'message_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'message_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'message_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sender_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'platoon_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'leader_id)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'robot_role) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'robot_role) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'predecessor_id)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'maneuver) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'maneuver) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Platooning>)))
  "Returns string type for a message object of type '<Platooning>"
  "cooperative_driving_networking/Platooning")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Platooning)))
  "Returns string type for a message object of type 'Platooning"
  "cooperative_driving_networking/Platooning")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Platooning>)))
  "Returns md5sum for a message object of type '<Platooning>"
  "ab1ba68fe20a61da2fb08021e8187972")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Platooning)))
  "Returns md5sum for a message object of type 'Platooning"
  "ab1ba68fe20a61da2fb08021e8187972")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Platooning>)))
  "Returns full string definition for message of type '<Platooning>"
  (cl:format cl:nil "# Used in the demo application for the token passing protocol~%~%# Common message header~%# Standard ROS message header~%Header header~%# Applicationwise uniquie identifier of this message content~%uint32 message_id~%# The id of the robot which sends this message~%uint8 sender_id~%# Specific data of this message type~%# TODO~%uint8 platoon_id~%# TODO~%uint8 leader_id~%# TODO~%string robot_role~%# TODO~%uint8 predecessor_id~%# TODO~%string maneuver~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Platooning)))
  "Returns full string definition for message of type 'Platooning"
  (cl:format cl:nil "# Used in the demo application for the token passing protocol~%~%# Common message header~%# Standard ROS message header~%Header header~%# Applicationwise uniquie identifier of this message content~%uint32 message_id~%# The id of the robot which sends this message~%uint8 sender_id~%# Specific data of this message type~%# TODO~%uint8 platoon_id~%# TODO~%uint8 leader_id~%# TODO~%string robot_role~%# TODO~%uint8 predecessor_id~%# TODO~%string maneuver~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Platooning>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     1
     1
     1
     4 (cl:length (cl:slot-value msg 'robot_role))
     1
     4 (cl:length (cl:slot-value msg 'maneuver))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Platooning>))
  "Converts a ROS message object to a list"
  (cl:list 'Platooning
    (cl:cons ':header (header msg))
    (cl:cons ':message_id (message_id msg))
    (cl:cons ':sender_id (sender_id msg))
    (cl:cons ':platoon_id (platoon_id msg))
    (cl:cons ':leader_id (leader_id msg))
    (cl:cons ':robot_role (robot_role msg))
    (cl:cons ':predecessor_id (predecessor_id msg))
    (cl:cons ':maneuver (maneuver msg))
))
