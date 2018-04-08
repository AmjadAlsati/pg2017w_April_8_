; Auto-generated. Do not edit!


(cl:in-package cooperative_driving_networking-srv)


;//! \htmlinclude BroadcastMessage-request.msg.html

(cl:defclass <BroadcastMessage-request> (roslisp-msg-protocol:ros-message)
  ((msg
    :reader msg
    :initarg :msg
    :type cl:string
    :initform ""))
)

(cl:defclass BroadcastMessage-request (<BroadcastMessage-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BroadcastMessage-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BroadcastMessage-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cooperative_driving_networking-srv:<BroadcastMessage-request> is deprecated: use cooperative_driving_networking-srv:BroadcastMessage-request instead.")))

(cl:ensure-generic-function 'msg-val :lambda-list '(m))
(cl:defmethod msg-val ((m <BroadcastMessage-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_networking-srv:msg-val is deprecated.  Use cooperative_driving_networking-srv:msg instead.")
  (msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BroadcastMessage-request>) ostream)
  "Serializes a message object of type '<BroadcastMessage-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'msg))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'msg))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BroadcastMessage-request>) istream)
  "Deserializes a message object of type '<BroadcastMessage-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'msg) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'msg) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BroadcastMessage-request>)))
  "Returns string type for a service object of type '<BroadcastMessage-request>"
  "cooperative_driving_networking/BroadcastMessageRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BroadcastMessage-request)))
  "Returns string type for a service object of type 'BroadcastMessage-request"
  "cooperative_driving_networking/BroadcastMessageRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BroadcastMessage-request>)))
  "Returns md5sum for a message object of type '<BroadcastMessage-request>"
  "7d96ed730776804754140b85e64c862e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BroadcastMessage-request)))
  "Returns md5sum for a message object of type 'BroadcastMessage-request"
  "7d96ed730776804754140b85e64c862e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BroadcastMessage-request>)))
  "Returns full string definition for message of type '<BroadcastMessage-request>"
  (cl:format cl:nil "string msg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BroadcastMessage-request)))
  "Returns full string definition for message of type 'BroadcastMessage-request"
  (cl:format cl:nil "string msg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BroadcastMessage-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BroadcastMessage-request>))
  "Converts a ROS message object to a list"
  (cl:list 'BroadcastMessage-request
    (cl:cons ':msg (msg msg))
))
;//! \htmlinclude BroadcastMessage-response.msg.html

(cl:defclass <BroadcastMessage-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass BroadcastMessage-response (<BroadcastMessage-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BroadcastMessage-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BroadcastMessage-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cooperative_driving_networking-srv:<BroadcastMessage-response> is deprecated: use cooperative_driving_networking-srv:BroadcastMessage-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BroadcastMessage-response>) ostream)
  "Serializes a message object of type '<BroadcastMessage-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BroadcastMessage-response>) istream)
  "Deserializes a message object of type '<BroadcastMessage-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BroadcastMessage-response>)))
  "Returns string type for a service object of type '<BroadcastMessage-response>"
  "cooperative_driving_networking/BroadcastMessageResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BroadcastMessage-response)))
  "Returns string type for a service object of type 'BroadcastMessage-response"
  "cooperative_driving_networking/BroadcastMessageResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BroadcastMessage-response>)))
  "Returns md5sum for a message object of type '<BroadcastMessage-response>"
  "7d96ed730776804754140b85e64c862e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BroadcastMessage-response)))
  "Returns md5sum for a message object of type 'BroadcastMessage-response"
  "7d96ed730776804754140b85e64c862e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BroadcastMessage-response>)))
  "Returns full string definition for message of type '<BroadcastMessage-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BroadcastMessage-response)))
  "Returns full string definition for message of type 'BroadcastMessage-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BroadcastMessage-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BroadcastMessage-response>))
  "Converts a ROS message object to a list"
  (cl:list 'BroadcastMessage-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'BroadcastMessage)))
  'BroadcastMessage-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'BroadcastMessage)))
  'BroadcastMessage-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BroadcastMessage)))
  "Returns string type for a service object of type '<BroadcastMessage>"
  "cooperative_driving_networking/BroadcastMessage")