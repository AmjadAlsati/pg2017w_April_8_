; Auto-generated. Do not edit!


(cl:in-package cooperative_driving_msgs-msg)


;//! \htmlinclude Direction.msg.html

(cl:defclass <Direction> (roslisp-msg-protocol:ros-message)
  ((direction
    :reader direction
    :initarg :direction
    :type cl:string
    :initform ""))
)

(cl:defclass Direction (<Direction>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Direction>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Direction)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cooperative_driving_msgs-msg:<Direction> is deprecated: use cooperative_driving_msgs-msg:Direction instead.")))

(cl:ensure-generic-function 'direction-val :lambda-list '(m))
(cl:defmethod direction-val ((m <Direction>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_msgs-msg:direction-val is deprecated.  Use cooperative_driving_msgs-msg:direction instead.")
  (direction m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Direction>) ostream)
  "Serializes a message object of type '<Direction>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'direction))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'direction))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Direction>) istream)
  "Deserializes a message object of type '<Direction>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'direction) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'direction) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Direction>)))
  "Returns string type for a message object of type '<Direction>"
  "cooperative_driving_msgs/Direction")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Direction)))
  "Returns string type for a message object of type 'Direction"
  "cooperative_driving_msgs/Direction")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Direction>)))
  "Returns md5sum for a message object of type '<Direction>"
  "326e9417def5db9a05a2704a4d8de15e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Direction)))
  "Returns md5sum for a message object of type 'Direction"
  "326e9417def5db9a05a2704a4d8de15e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Direction>)))
  "Returns full string definition for message of type '<Direction>"
  (cl:format cl:nil "## Define a direction of a detected crossing, i.e., a turn that can possibly be taken~%~%# Possible turn at a crossing ('straight', 'left', or 'right')~%string direction~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Direction)))
  "Returns full string definition for message of type 'Direction"
  (cl:format cl:nil "## Define a direction of a detected crossing, i.e., a turn that can possibly be taken~%~%# Possible turn at a crossing ('straight', 'left', or 'right')~%string direction~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Direction>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'direction))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Direction>))
  "Converts a ROS message object to a list"
  (cl:list 'Direction
    (cl:cons ':direction (direction msg))
))
