; Auto-generated. Do not edit!


(cl:in-package cooperative_driving_hardware-msg)


;//! \htmlinclude LedCommand.msg.html

(cl:defclass <LedCommand> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (values
    :reader values
    :initarg :values
    :type (cl:vector cooperative_driving_hardware-msg:LedValue)
   :initform (cl:make-array 0 :element-type 'cooperative_driving_hardware-msg:LedValue :initial-element (cl:make-instance 'cooperative_driving_hardware-msg:LedValue))))
)

(cl:defclass LedCommand (<LedCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LedCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LedCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cooperative_driving_hardware-msg:<LedCommand> is deprecated: use cooperative_driving_hardware-msg:LedCommand instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <LedCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_hardware-msg:header-val is deprecated.  Use cooperative_driving_hardware-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'values-val :lambda-list '(m))
(cl:defmethod values-val ((m <LedCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_hardware-msg:values-val is deprecated.  Use cooperative_driving_hardware-msg:values instead.")
  (values m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LedCommand>) ostream)
  "Serializes a message object of type '<LedCommand>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'values))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'values))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LedCommand>) istream)
  "Deserializes a message object of type '<LedCommand>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'values) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'values)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'cooperative_driving_hardware-msg:LedValue))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LedCommand>)))
  "Returns string type for a message object of type '<LedCommand>"
  "cooperative_driving_hardware/LedCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LedCommand)))
  "Returns string type for a message object of type 'LedCommand"
  "cooperative_driving_hardware/LedCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LedCommand>)))
  "Returns md5sum for a message object of type '<LedCommand>"
  "7149f6e7328bbb637f466762bbc9f089")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LedCommand)))
  "Returns md5sum for a message object of type 'LedCommand"
  "7149f6e7328bbb637f466762bbc9f089")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LedCommand>)))
  "Returns full string definition for message of type '<LedCommand>"
  (cl:format cl:nil "# Used for flashing LEDs~%~%# Standard ROS message header~%Header header~%# An array of LedValue messages which represent the LED values to set~%LedValue[] values~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: cooperative_driving_hardware/LedValue~%# Custom message type represent a reading from a distance sensor used for reading and writing~%~%# The frame_id (name) of this LED~%string frame_id~%# The current value of this LED~%# Values in [0, 1]~%float32 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LedCommand)))
  "Returns full string definition for message of type 'LedCommand"
  (cl:format cl:nil "# Used for flashing LEDs~%~%# Standard ROS message header~%Header header~%# An array of LedValue messages which represent the LED values to set~%LedValue[] values~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: cooperative_driving_hardware/LedValue~%# Custom message type represent a reading from a distance sensor used for reading and writing~%~%# The frame_id (name) of this LED~%string frame_id~%# The current value of this LED~%# Values in [0, 1]~%float32 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LedCommand>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'values) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LedCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'LedCommand
    (cl:cons ':header (header msg))
    (cl:cons ':values (values msg))
))
