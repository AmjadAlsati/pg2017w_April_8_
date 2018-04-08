; Auto-generated. Do not edit!


(cl:in-package cooperative_driving_hardware-msg)


;//! \htmlinclude DistanceValue.msg.html

(cl:defclass <DistanceValue> (roslisp-msg-protocol:ros-message)
  ((frame_id
    :reader frame_id
    :initarg :frame_id
    :type cl:string
    :initform "")
   (distance
    :reader distance
    :initarg :distance
    :type cl:float
    :initform 0.0))
)

(cl:defclass DistanceValue (<DistanceValue>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DistanceValue>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DistanceValue)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cooperative_driving_hardware-msg:<DistanceValue> is deprecated: use cooperative_driving_hardware-msg:DistanceValue instead.")))

(cl:ensure-generic-function 'frame_id-val :lambda-list '(m))
(cl:defmethod frame_id-val ((m <DistanceValue>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_hardware-msg:frame_id-val is deprecated.  Use cooperative_driving_hardware-msg:frame_id instead.")
  (frame_id m))

(cl:ensure-generic-function 'distance-val :lambda-list '(m))
(cl:defmethod distance-val ((m <DistanceValue>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_hardware-msg:distance-val is deprecated.  Use cooperative_driving_hardware-msg:distance instead.")
  (distance m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DistanceValue>) ostream)
  "Serializes a message object of type '<DistanceValue>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'frame_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'frame_id))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DistanceValue>) istream)
  "Deserializes a message object of type '<DistanceValue>"
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
    (cl:setf (cl:slot-value msg 'distance) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DistanceValue>)))
  "Returns string type for a message object of type '<DistanceValue>"
  "cooperative_driving_hardware/DistanceValue")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DistanceValue)))
  "Returns string type for a message object of type 'DistanceValue"
  "cooperative_driving_hardware/DistanceValue")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DistanceValue>)))
  "Returns md5sum for a message object of type '<DistanceValue>"
  "6464eba0f0ad6d8ca3357c316d6b0ed2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DistanceValue)))
  "Returns md5sum for a message object of type 'DistanceValue"
  "6464eba0f0ad6d8ca3357c316d6b0ed2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DistanceValue>)))
  "Returns full string definition for message of type '<DistanceValue>"
  (cl:format cl:nil "# Custom message type represent a reading from a distance sensor used for reading~%~%# The frame_id (name) of this sensor~%string frame_id~%# The reading of this sensor~%# Values in m~%float32 distance~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DistanceValue)))
  "Returns full string definition for message of type 'DistanceValue"
  (cl:format cl:nil "# Custom message type represent a reading from a distance sensor used for reading~%~%# The frame_id (name) of this sensor~%string frame_id~%# The reading of this sensor~%# Values in m~%float32 distance~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DistanceValue>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'frame_id))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DistanceValue>))
  "Converts a ROS message object to a list"
  (cl:list 'DistanceValue
    (cl:cons ':frame_id (frame_id msg))
    (cl:cons ':distance (distance msg))
))
