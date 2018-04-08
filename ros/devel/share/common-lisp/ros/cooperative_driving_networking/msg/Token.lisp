; Auto-generated. Do not edit!


(cl:in-package cooperative_driving_networking-msg)


;//! \htmlinclude Token.msg.html

(cl:defclass <Token> (roslisp-msg-protocol:ros-message)
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
   (token_id
    :reader token_id
    :initarg :token_id
    :type cl:integer
    :initform 0))
)

(cl:defclass Token (<Token>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Token>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Token)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cooperative_driving_networking-msg:<Token> is deprecated: use cooperative_driving_networking-msg:Token instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Token>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_networking-msg:header-val is deprecated.  Use cooperative_driving_networking-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'message_id-val :lambda-list '(m))
(cl:defmethod message_id-val ((m <Token>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_networking-msg:message_id-val is deprecated.  Use cooperative_driving_networking-msg:message_id instead.")
  (message_id m))

(cl:ensure-generic-function 'sender_id-val :lambda-list '(m))
(cl:defmethod sender_id-val ((m <Token>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_networking-msg:sender_id-val is deprecated.  Use cooperative_driving_networking-msg:sender_id instead.")
  (sender_id m))

(cl:ensure-generic-function 'token_id-val :lambda-list '(m))
(cl:defmethod token_id-val ((m <Token>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_networking-msg:token_id-val is deprecated.  Use cooperative_driving_networking-msg:token_id instead.")
  (token_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Token>) ostream)
  "Serializes a message object of type '<Token>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'message_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'message_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'message_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'message_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sender_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'token_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'token_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'token_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'token_id)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Token>) istream)
  "Deserializes a message object of type '<Token>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'message_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'message_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'message_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'message_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sender_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'token_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'token_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'token_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'token_id)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Token>)))
  "Returns string type for a message object of type '<Token>"
  "cooperative_driving_networking/Token")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Token)))
  "Returns string type for a message object of type 'Token"
  "cooperative_driving_networking/Token")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Token>)))
  "Returns md5sum for a message object of type '<Token>"
  "2cf72de3eab0cac539f4018d2c17b401")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Token)))
  "Returns md5sum for a message object of type 'Token"
  "2cf72de3eab0cac539f4018d2c17b401")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Token>)))
  "Returns full string definition for message of type '<Token>"
  (cl:format cl:nil "# Used in the demo application for the token passing protocol~%~%# Common message header~%# Standard ROS message header~%Header header~%# Applicationwise uniquie identifier of this message content~%uint32 message_id~%# The id of the robot which sends this message~%uint8 sender_id~%# Specific data of this message type~%# The token forwared by this messages represented via its id~%uint32 token_id~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Token)))
  "Returns full string definition for message of type 'Token"
  (cl:format cl:nil "# Used in the demo application for the token passing protocol~%~%# Common message header~%# Standard ROS message header~%Header header~%# Applicationwise uniquie identifier of this message content~%uint32 message_id~%# The id of the robot which sends this message~%uint8 sender_id~%# Specific data of this message type~%# The token forwared by this messages represented via its id~%uint32 token_id~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Token>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Token>))
  "Converts a ROS message object to a list"
  (cl:list 'Token
    (cl:cons ':header (header msg))
    (cl:cons ':message_id (message_id msg))
    (cl:cons ':sender_id (sender_id msg))
    (cl:cons ':token_id (token_id msg))
))
