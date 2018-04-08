; Auto-generated. Do not edit!


(cl:in-package cooperative_driving_hardware-msg)


;//! \htmlinclude LedValue.msg.html

(cl:defclass <LedValue> (roslisp-msg-protocol:ros-message)
  ((frame_id
    :reader frame_id
    :initarg :frame_id
    :type cl:string
    :initform "")
   (value
    :reader value
    :initarg :value
    :type cl:float
    :initform 0.0))
)

(cl:defclass LedValue (<LedValue>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LedValue>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LedValue)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cooperative_driving_hardware-msg:<LedValue> is deprecated: use cooperative_driving_hardware-msg:LedValue instead.")))

(cl:ensure-generic-function 'frame_id-val :lambda-list '(m))
(cl:defmethod frame_id-val ((m <LedValue>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_hardware-msg:frame_id-val is deprecated.  Use cooperative_driving_hardware-msg:frame_id instead.")
  (frame_id m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <LedValue>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_hardware-msg:value-val is deprecated.  Use cooperative_driving_hardware-msg:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LedValue>) ostream)
  "Serializes a message object of type '<LedValue>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'frame_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'frame_id))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LedValue>) istream)
  "Deserializes a message object of type '<LedValue>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'frame_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'frame_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'value) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LedValue>)))
  "Returns string type for a message object of type '<LedValue>"
  "cooperative_driving_hardware/LedValue")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LedValue)))
  "Returns string type for a message object of type 'LedValue"
  "cooperative_driving_hardware/LedValue")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LedValue>)))
  "Returns md5sum for a message object of type '<LedValue>"
  "6a0c35f631f3f53c7b19b24b9d87c563")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LedValue)))
  "Returns md5sum for a message object of type 'LedValue"
  "6a0c35f631f3f53c7b19b24b9d87c563")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LedValue>)))
  "Returns full string definition for message of type '<LedValue>"
  (cl:format cl:nil "# Custom message type represent a reading from a distance sensor used for reading and writing~%~%# The frame_id (name) of this LED~%string frame_id~%# The current value of this LED~%# Values in [0, 1]~%float32 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LedValue)))
  "Returns full string definition for message of type 'LedValue"
  (cl:format cl:nil "# Custom message type represent a reading from a distance sensor used for reading and writing~%~%# The frame_id (name) of this LED~%string frame_id~%# The current value of this LED~%# Values in [0, 1]~%float32 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LedValue>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'frame_id))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LedValue>))
  "Converts a ROS message object to a list"
  (cl:list 'LedValue
    (cl:cons ':frame_id (frame_id msg))
    (cl:cons ':value (value msg))
))
