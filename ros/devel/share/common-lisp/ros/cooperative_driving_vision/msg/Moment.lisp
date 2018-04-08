; Auto-generated. Do not edit!


(cl:in-package cooperative_driving_vision-msg)


;//! \htmlinclude Moment.msg.html

(cl:defclass <Moment> (roslisp-msg-protocol:ros-message)
  ((m00
    :reader m00
    :initarg :m00
    :type cl:float
    :initform 0.0)
   (m10
    :reader m10
    :initarg :m10
    :type cl:float
    :initform 0.0)
   (m01
    :reader m01
    :initarg :m01
    :type cl:float
    :initform 0.0)
   (m11
    :reader m11
    :initarg :m11
    :type cl:float
    :initform 0.0)
   (m20
    :reader m20
    :initarg :m20
    :type cl:float
    :initform 0.0)
   (m02
    :reader m02
    :initarg :m02
    :type cl:float
    :initform 0.0)
   (m21
    :reader m21
    :initarg :m21
    :type cl:float
    :initform 0.0)
   (m12
    :reader m12
    :initarg :m12
    :type cl:float
    :initform 0.0)
   (m30
    :reader m30
    :initarg :m30
    :type cl:float
    :initform 0.0)
   (m03
    :reader m03
    :initarg :m03
    :type cl:float
    :initform 0.0))
)

(cl:defclass Moment (<Moment>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Moment>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Moment)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cooperative_driving_vision-msg:<Moment> is deprecated: use cooperative_driving_vision-msg:Moment instead.")))

(cl:ensure-generic-function 'm00-val :lambda-list '(m))
(cl:defmethod m00-val ((m <Moment>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_vision-msg:m00-val is deprecated.  Use cooperative_driving_vision-msg:m00 instead.")
  (m00 m))

(cl:ensure-generic-function 'm10-val :lambda-list '(m))
(cl:defmethod m10-val ((m <Moment>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_vision-msg:m10-val is deprecated.  Use cooperative_driving_vision-msg:m10 instead.")
  (m10 m))

(cl:ensure-generic-function 'm01-val :lambda-list '(m))
(cl:defmethod m01-val ((m <Moment>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_vision-msg:m01-val is deprecated.  Use cooperative_driving_vision-msg:m01 instead.")
  (m01 m))

(cl:ensure-generic-function 'm11-val :lambda-list '(m))
(cl:defmethod m11-val ((m <Moment>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_vision-msg:m11-val is deprecated.  Use cooperative_driving_vision-msg:m11 instead.")
  (m11 m))

(cl:ensure-generic-function 'm20-val :lambda-list '(m))
(cl:defmethod m20-val ((m <Moment>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_vision-msg:m20-val is deprecated.  Use cooperative_driving_vision-msg:m20 instead.")
  (m20 m))

(cl:ensure-generic-function 'm02-val :lambda-list '(m))
(cl:defmethod m02-val ((m <Moment>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_vision-msg:m02-val is deprecated.  Use cooperative_driving_vision-msg:m02 instead.")
  (m02 m))

(cl:ensure-generic-function 'm21-val :lambda-list '(m))
(cl:defmethod m21-val ((m <Moment>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_vision-msg:m21-val is deprecated.  Use cooperative_driving_vision-msg:m21 instead.")
  (m21 m))

(cl:ensure-generic-function 'm12-val :lambda-list '(m))
(cl:defmethod m12-val ((m <Moment>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_vision-msg:m12-val is deprecated.  Use cooperative_driving_vision-msg:m12 instead.")
  (m12 m))

(cl:ensure-generic-function 'm30-val :lambda-list '(m))
(cl:defmethod m30-val ((m <Moment>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_vision-msg:m30-val is deprecated.  Use cooperative_driving_vision-msg:m30 instead.")
  (m30 m))

(cl:ensure-generic-function 'm03-val :lambda-list '(m))
(cl:defmethod m03-val ((m <Moment>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_vision-msg:m03-val is deprecated.  Use cooperative_driving_vision-msg:m03 instead.")
  (m03 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Moment>) ostream)
  "Serializes a message object of type '<Moment>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'm00))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'm10))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'm01))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'm11))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'm20))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'm02))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'm21))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'm12))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'm30))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'm03))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Moment>) istream)
  "Deserializes a message object of type '<Moment>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'm00) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'm10) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'm01) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'm11) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'm20) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'm02) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'm21) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'm12) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'm30) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'm03) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Moment>)))
  "Returns string type for a message object of type '<Moment>"
  "cooperative_driving_vision/Moment")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Moment)))
  "Returns string type for a message object of type 'Moment"
  "cooperative_driving_vision/Moment")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Moment>)))
  "Returns md5sum for a message object of type '<Moment>"
  "15ca49fd130f761e715cfaf6f1985ad7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Moment)))
  "Returns md5sum for a message object of type 'Moment"
  "15ca49fd130f761e715cfaf6f1985ad7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Moment>)))
  "Returns full string definition for message of type '<Moment>"
  (cl:format cl:nil "# Custom message type to represent a geometrical moment~%~%# (0, 0)th raw moment, i.e. number of pixels~%float32 m00~%# (1, 0)th raw moment~%float32 m10~%# (0, 1)th raw moment~%float32 m01~%# (1, 1)th raw moment~%float32 m11~%# (2, 0)th raw moment~%float32 m20~%# (0, 2)th raw moment~%float32 m02~%# (2, 1)th raw moment~%float32 m21~%# (1, 2)th raw moment~%float32 m12~%# (3, 0)th raw moment~%float32 m30~%# (0, 3)th raw moment~%float32 m03~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Moment)))
  "Returns full string definition for message of type 'Moment"
  (cl:format cl:nil "# Custom message type to represent a geometrical moment~%~%# (0, 0)th raw moment, i.e. number of pixels~%float32 m00~%# (1, 0)th raw moment~%float32 m10~%# (0, 1)th raw moment~%float32 m01~%# (1, 1)th raw moment~%float32 m11~%# (2, 0)th raw moment~%float32 m20~%# (0, 2)th raw moment~%float32 m02~%# (2, 1)th raw moment~%float32 m21~%# (1, 2)th raw moment~%float32 m12~%# (3, 0)th raw moment~%float32 m30~%# (0, 3)th raw moment~%float32 m03~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Moment>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Moment>))
  "Converts a ROS message object to a list"
  (cl:list 'Moment
    (cl:cons ':m00 (m00 msg))
    (cl:cons ':m10 (m10 msg))
    (cl:cons ':m01 (m01 msg))
    (cl:cons ':m11 (m11 msg))
    (cl:cons ':m20 (m20 msg))
    (cl:cons ':m02 (m02 msg))
    (cl:cons ':m21 (m21 msg))
    (cl:cons ':m12 (m12 msg))
    (cl:cons ':m30 (m30 msg))
    (cl:cons ':m03 (m03 msg))
))
