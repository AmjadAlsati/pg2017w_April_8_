; Auto-generated. Do not edit!


(cl:in-package cooperative_driving_vision-msg)


;//! \htmlinclude Region.msg.html

(cl:defclass <Region> (roslisp-msg-protocol:ros-message)
  ((color
    :reader color
    :initarg :color
    :type std_msgs-msg:ColorRGBA
    :initform (cl:make-instance 'std_msgs-msg:ColorRGBA))
   (moment
    :reader moment
    :initarg :moment
    :type cooperative_driving_vision-msg:Moment
    :initform (cl:make-instance 'cooperative_driving_vision-msg:Moment)))
)

(cl:defclass Region (<Region>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Region>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Region)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cooperative_driving_vision-msg:<Region> is deprecated: use cooperative_driving_vision-msg:Region instead.")))

(cl:ensure-generic-function 'color-val :lambda-list '(m))
(cl:defmethod color-val ((m <Region>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_vision-msg:color-val is deprecated.  Use cooperative_driving_vision-msg:color instead.")
  (color m))

(cl:ensure-generic-function 'moment-val :lambda-list '(m))
(cl:defmethod moment-val ((m <Region>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_vision-msg:moment-val is deprecated.  Use cooperative_driving_vision-msg:moment instead.")
  (moment m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Region>) ostream)
  "Serializes a message object of type '<Region>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'color) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'moment) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Region>) istream)
  "Deserializes a message object of type '<Region>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'color) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'moment) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Region>)))
  "Returns string type for a message object of type '<Region>"
  "cooperative_driving_vision/Region")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Region)))
  "Returns string type for a message object of type 'Region"
  "cooperative_driving_vision/Region")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Region>)))
  "Returns md5sum for a message object of type '<Region>"
  "b981b501a8a1ad1a59b87231a82a1888")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Region)))
  "Returns md5sum for a message object of type 'Region"
  "b981b501a8a1ad1a59b87231a82a1888")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Region>)))
  "Returns full string definition for message of type '<Region>"
  (cl:format cl:nil "# Custom message type to represent an extracted region used for publishing~%~%# The region's average color~%std_msgs/ColorRGBA color~%# The geometrical moment describing the extents of the region~%Moment moment~%~%================================================================================~%MSG: std_msgs/ColorRGBA~%float32 r~%float32 g~%float32 b~%float32 a~%~%================================================================================~%MSG: cooperative_driving_vision/Moment~%# Custom message type to represent a geometrical moment~%~%# (0, 0)th raw moment, i.e. number of pixels~%float32 m00~%# (1, 0)th raw moment~%float32 m10~%# (0, 1)th raw moment~%float32 m01~%# (1, 1)th raw moment~%float32 m11~%# (2, 0)th raw moment~%float32 m20~%# (0, 2)th raw moment~%float32 m02~%# (2, 1)th raw moment~%float32 m21~%# (1, 2)th raw moment~%float32 m12~%# (3, 0)th raw moment~%float32 m30~%# (0, 3)th raw moment~%float32 m03~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Region)))
  "Returns full string definition for message of type 'Region"
  (cl:format cl:nil "# Custom message type to represent an extracted region used for publishing~%~%# The region's average color~%std_msgs/ColorRGBA color~%# The geometrical moment describing the extents of the region~%Moment moment~%~%================================================================================~%MSG: std_msgs/ColorRGBA~%float32 r~%float32 g~%float32 b~%float32 a~%~%================================================================================~%MSG: cooperative_driving_vision/Moment~%# Custom message type to represent a geometrical moment~%~%# (0, 0)th raw moment, i.e. number of pixels~%float32 m00~%# (1, 0)th raw moment~%float32 m10~%# (0, 1)th raw moment~%float32 m01~%# (1, 1)th raw moment~%float32 m11~%# (2, 0)th raw moment~%float32 m20~%# (0, 2)th raw moment~%float32 m02~%# (2, 1)th raw moment~%float32 m21~%# (1, 2)th raw moment~%float32 m12~%# (3, 0)th raw moment~%float32 m30~%# (0, 3)th raw moment~%float32 m03~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Region>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'color))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'moment))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Region>))
  "Converts a ROS message object to a list"
  (cl:list 'Region
    (cl:cons ':color (color msg))
    (cl:cons ':moment (moment msg))
))
