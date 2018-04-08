; Auto-generated. Do not edit!


(cl:in-package cooperative_driving_logic-srv)


;//! \htmlinclude ChangeState-request.msg.html

(cl:defclass <ChangeState-request> (roslisp-msg-protocol:ros-message)
  ((target_state
    :reader target_state
    :initarg :target_state
    :type cl:string
    :initform "")
   (tag_id_to_follow
    :reader tag_id_to_follow
    :initarg :tag_id_to_follow
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ChangeState-request (<ChangeState-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ChangeState-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ChangeState-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cooperative_driving_logic-srv:<ChangeState-request> is deprecated: use cooperative_driving_logic-srv:ChangeState-request instead.")))

(cl:ensure-generic-function 'target_state-val :lambda-list '(m))
(cl:defmethod target_state-val ((m <ChangeState-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_logic-srv:target_state-val is deprecated.  Use cooperative_driving_logic-srv:target_state instead.")
  (target_state m))

(cl:ensure-generic-function 'tag_id_to_follow-val :lambda-list '(m))
(cl:defmethod tag_id_to_follow-val ((m <ChangeState-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_logic-srv:tag_id_to_follow-val is deprecated.  Use cooperative_driving_logic-srv:tag_id_to_follow instead.")
  (tag_id_to_follow m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ChangeState-request>)))
    "Constants for message type '<ChangeState-request>"
  '((:REMOTE_CONTROL . remote_control)
    (:FOLLOW_BLOB . follow_blob)
    (:FOLLOW_LINE . follow_line)
    (:PLATOONING . platooning)
    (:DYNAMIC_FOLLOW_LINE . dynamic_follow_line)
    (:IDLE . idle))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ChangeState-request)))
    "Constants for message type 'ChangeState-request"
  '((:REMOTE_CONTROL . remote_control)
    (:FOLLOW_BLOB . follow_blob)
    (:FOLLOW_LINE . follow_line)
    (:PLATOONING . platooning)
    (:DYNAMIC_FOLLOW_LINE . dynamic_follow_line)
    (:IDLE . idle))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ChangeState-request>) ostream)
  "Serializes a message object of type '<ChangeState-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'target_state))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'target_state))
  (cl:let* ((signed (cl:slot-value msg 'tag_id_to_follow)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ChangeState-request>) istream)
  "Deserializes a message object of type '<ChangeState-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'target_state) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'target_state) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'tag_id_to_follow) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ChangeState-request>)))
  "Returns string type for a service object of type '<ChangeState-request>"
  "cooperative_driving_logic/ChangeStateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ChangeState-request)))
  "Returns string type for a service object of type 'ChangeState-request"
  "cooperative_driving_logic/ChangeStateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ChangeState-request>)))
  "Returns md5sum for a message object of type '<ChangeState-request>"
  "47b07bdab417ccb3c955c49a6d18057b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ChangeState-request)))
  "Returns md5sum for a message object of type 'ChangeState-request"
  "47b07bdab417ccb3c955c49a6d18057b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ChangeState-request>)))
  "Returns full string definition for message of type '<ChangeState-request>"
  (cl:format cl:nil "~%string REMOTE_CONTROL=remote_control~%string FOLLOW_BLOB=follow_blob~%string FOLLOW_LINE=follow_line~%string PLATOONING=platooning~%string DYNAMIC_FOLLOW_LINE=dynamic_follow_line~%string IDLE=idle~%~%string target_state~%int8 tag_id_to_follow~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ChangeState-request)))
  "Returns full string definition for message of type 'ChangeState-request"
  (cl:format cl:nil "~%string REMOTE_CONTROL=remote_control~%string FOLLOW_BLOB=follow_blob~%string FOLLOW_LINE=follow_line~%string PLATOONING=platooning~%string DYNAMIC_FOLLOW_LINE=dynamic_follow_line~%string IDLE=idle~%~%string target_state~%int8 tag_id_to_follow~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ChangeState-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'target_state))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ChangeState-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ChangeState-request
    (cl:cons ':target_state (target_state msg))
    (cl:cons ':tag_id_to_follow (tag_id_to_follow msg))
))
;//! \htmlinclude ChangeState-response.msg.html

(cl:defclass <ChangeState-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ChangeState-response (<ChangeState-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ChangeState-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ChangeState-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cooperative_driving_logic-srv:<ChangeState-response> is deprecated: use cooperative_driving_logic-srv:ChangeState-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ChangeState-response>) ostream)
  "Serializes a message object of type '<ChangeState-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ChangeState-response>) istream)
  "Deserializes a message object of type '<ChangeState-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ChangeState-response>)))
  "Returns string type for a service object of type '<ChangeState-response>"
  "cooperative_driving_logic/ChangeStateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ChangeState-response)))
  "Returns string type for a service object of type 'ChangeState-response"
  "cooperative_driving_logic/ChangeStateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ChangeState-response>)))
  "Returns md5sum for a message object of type '<ChangeState-response>"
  "47b07bdab417ccb3c955c49a6d18057b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ChangeState-response)))
  "Returns md5sum for a message object of type 'ChangeState-response"
  "47b07bdab417ccb3c955c49a6d18057b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ChangeState-response>)))
  "Returns full string definition for message of type '<ChangeState-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ChangeState-response)))
  "Returns full string definition for message of type 'ChangeState-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ChangeState-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ChangeState-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ChangeState-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ChangeState)))
  'ChangeState-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ChangeState)))
  'ChangeState-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ChangeState)))
  "Returns string type for a service object of type '<ChangeState>"
  "cooperative_driving_logic/ChangeState")