; Auto-generated. Do not edit!


(cl:in-package cooperative_driving_networking-msg)


;//! \htmlinclude EmergencyBrake.msg.html

(cl:defclass <EmergencyBrake> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (sender_id
    :reader sender_id
    :initarg :sender_id
    :type cl:fixnum
    :initform 0)
   (enable
    :reader enable
    :initarg :enable
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass EmergencyBrake (<EmergencyBrake>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EmergencyBrake>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EmergencyBrake)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cooperative_driving_networking-msg:<EmergencyBrake> is deprecated: use cooperative_driving_networking-msg:EmergencyBrake instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <EmergencyBrake>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_networking-msg:header-val is deprecated.  Use cooperative_driving_networking-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'sender_id-val :lambda-list '(m))
(cl:defmethod sender_id-val ((m <EmergencyBrake>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_networking-msg:sender_id-val is deprecated.  Use cooperative_driving_networking-msg:sender_id instead.")
  (sender_id m))

(cl:ensure-generic-function 'enable-val :lambda-list '(m))
(cl:defmethod enable-val ((m <EmergencyBrake>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_networking-msg:enable-val is deprecated.  Use cooperative_driving_networking-msg:enable instead.")
  (enable m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EmergencyBrake>) ostream)
  "Serializes a message object of type '<EmergencyBrake>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sender_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'enable) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EmergencyBrake>) istream)
  "Deserializes a message object of type '<EmergencyBrake>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sender_id)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'enable) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EmergencyBrake>)))
  "Returns string type for a message object of type '<EmergencyBrake>"
  "cooperative_driving_networking/EmergencyBrake")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EmergencyBrake)))
  "Returns string type for a message object of type 'EmergencyBrake"
  "cooperative_driving_networking/EmergencyBrake")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EmergencyBrake>)))
  "Returns md5sum for a message object of type '<EmergencyBrake>"
  "9f70b5ab2cb5f15d7d35f30d081697ad")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EmergencyBrake)))
  "Returns md5sum for a message object of type 'EmergencyBrake"
  "9f70b5ab2cb5f15d7d35f30d081697ad")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EmergencyBrake>)))
  "Returns full string definition for message of type '<EmergencyBrake>"
  (cl:format cl:nil "# Used in the demo application for the token passing protocol~%~%# Common message header~%# Standard ROS message header~%Header header~%# The id of the robot which sends this message~%uint8 sender_id~%# Specific data of this message type~%# Flag to indicate whether an emergency braking situation occured or is resolved~%bool enable~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EmergencyBrake)))
  "Returns full string definition for message of type 'EmergencyBrake"
  (cl:format cl:nil "# Used in the demo application for the token passing protocol~%~%# Common message header~%# Standard ROS message header~%Header header~%# The id of the robot which sends this message~%uint8 sender_id~%# Specific data of this message type~%# Flag to indicate whether an emergency braking situation occured or is resolved~%bool enable~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EmergencyBrake>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EmergencyBrake>))
  "Converts a ROS message object to a list"
  (cl:list 'EmergencyBrake
    (cl:cons ':header (header msg))
    (cl:cons ':sender_id (sender_id msg))
    (cl:cons ':enable (enable msg))
))
