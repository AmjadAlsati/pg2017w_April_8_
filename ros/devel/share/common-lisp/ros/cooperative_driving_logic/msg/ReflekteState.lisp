; Auto-generated. Do not edit!


(cl:in-package cooperative_driving_logic-msg)


;//! \htmlinclude ReflekteState.msg.html

(cl:defclass <ReflekteState> (roslisp-msg-protocol:ros-message)
  ((state
    :reader state
    :initarg :state
    :type cl:string
    :initform "")
   (node_to_follow
    :reader node_to_follow
    :initarg :node_to_follow
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ReflekteState (<ReflekteState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReflekteState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReflekteState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cooperative_driving_logic-msg:<ReflekteState> is deprecated: use cooperative_driving_logic-msg:ReflekteState instead.")))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <ReflekteState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_logic-msg:state-val is deprecated.  Use cooperative_driving_logic-msg:state instead.")
  (state m))

(cl:ensure-generic-function 'node_to_follow-val :lambda-list '(m))
(cl:defmethod node_to_follow-val ((m <ReflekteState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_logic-msg:node_to_follow-val is deprecated.  Use cooperative_driving_logic-msg:node_to_follow instead.")
  (node_to_follow m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReflekteState>) ostream)
  "Serializes a message object of type '<ReflekteState>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'state))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'state))
  (cl:let* ((signed (cl:slot-value msg 'node_to_follow)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReflekteState>) istream)
  "Deserializes a message object of type '<ReflekteState>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'state) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'state) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'node_to_follow) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReflekteState>)))
  "Returns string type for a message object of type '<ReflekteState>"
  "cooperative_driving_logic/ReflekteState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReflekteState)))
  "Returns string type for a message object of type 'ReflekteState"
  "cooperative_driving_logic/ReflekteState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReflekteState>)))
  "Returns md5sum for a message object of type '<ReflekteState>"
  "43cc3c773585d3b826a620cbff7f3808")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReflekteState)))
  "Returns md5sum for a message object of type 'ReflekteState"
  "43cc3c773585d3b826a620cbff7f3808")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReflekteState>)))
  "Returns full string definition for message of type '<ReflekteState>"
  (cl:format cl:nil "# Used to publish the current state of the reactive behavior in the Reflekte node~%~%# The current state of the reactive behavior~%string state~%int8 node_to_follow~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReflekteState)))
  "Returns full string definition for message of type 'ReflekteState"
  (cl:format cl:nil "# Used to publish the current state of the reactive behavior in the Reflekte node~%~%# The current state of the reactive behavior~%string state~%int8 node_to_follow~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReflekteState>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'state))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReflekteState>))
  "Converts a ROS message object to a list"
  (cl:list 'ReflekteState
    (cl:cons ':state (state msg))
    (cl:cons ':node_to_follow (node_to_follow msg))
))
