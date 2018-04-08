; Auto-generated. Do not edit!


(cl:in-package cooperative_driving_msgs-msg)


;//! \htmlinclude Directions.msg.html

(cl:defclass <Directions> (roslisp-msg-protocol:ros-message)
  ((directions
    :reader directions
    :initarg :directions
    :type (cl:vector cooperative_driving_msgs-msg:Direction)
   :initform (cl:make-array 0 :element-type 'cooperative_driving_msgs-msg:Direction :initial-element (cl:make-instance 'cooperative_driving_msgs-msg:Direction))))
)

(cl:defclass Directions (<Directions>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Directions>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Directions)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cooperative_driving_msgs-msg:<Directions> is deprecated: use cooperative_driving_msgs-msg:Directions instead.")))

(cl:ensure-generic-function 'directions-val :lambda-list '(m))
(cl:defmethod directions-val ((m <Directions>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_msgs-msg:directions-val is deprecated.  Use cooperative_driving_msgs-msg:directions instead.")
  (directions m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Directions>) ostream)
  "Serializes a message object of type '<Directions>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'directions))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'directions))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Directions>) istream)
  "Deserializes a message object of type '<Directions>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'directions) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'directions)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'cooperative_driving_msgs-msg:Direction))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Directions>)))
  "Returns string type for a message object of type '<Directions>"
  "cooperative_driving_msgs/Directions")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Directions)))
  "Returns string type for a message object of type 'Directions"
  "cooperative_driving_msgs/Directions")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Directions>)))
  "Returns md5sum for a message object of type '<Directions>"
  "f467a95e7546b9935ac1d69c0341e385")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Directions)))
  "Returns md5sum for a message object of type 'Directions"
  "f467a95e7546b9935ac1d69c0341e385")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Directions>)))
  "Returns full string definition for message of type '<Directions>"
  (cl:format cl:nil "## List all directions of a detected crossing~%~%# Array of all directions of a crossing ~%Direction[] directions~%================================================================================~%MSG: cooperative_driving_msgs/Direction~%## Define a direction of a detected crossing, i.e., a turn that can possibly be taken~%~%# Possible turn at a crossing ('straight', 'left', or 'right')~%string direction~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Directions)))
  "Returns full string definition for message of type 'Directions"
  (cl:format cl:nil "## List all directions of a detected crossing~%~%# Array of all directions of a crossing ~%Direction[] directions~%================================================================================~%MSG: cooperative_driving_msgs/Direction~%## Define a direction of a detected crossing, i.e., a turn that can possibly be taken~%~%# Possible turn at a crossing ('straight', 'left', or 'right')~%string direction~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Directions>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'directions) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Directions>))
  "Converts a ROS message object to a list"
  (cl:list 'Directions
    (cl:cons ':directions (directions msg))
))
