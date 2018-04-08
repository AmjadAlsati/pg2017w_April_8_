; Auto-generated. Do not edit!


(cl:in-package cooperative_driving_vision-msg)


;//! \htmlinclude Features.msg.html

(cl:defclass <Features> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (image_width
    :reader image_width
    :initarg :image_width
    :type cl:fixnum
    :initform 0)
   (image_height
    :reader image_height
    :initarg :image_height
    :type cl:fixnum
    :initform 0)
   (Hlines
    :reader Hlines
    :initarg :Hlines
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point)))
   (Vlines
    :reader Vlines
    :initarg :Vlines
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point)))
   (PFPS
    :reader PFPS
    :initarg :PFPS
    :type cl:fixnum
    :initform 0)
   (regions
    :reader regions
    :initarg :regions
    :type (cl:vector cooperative_driving_vision-msg:Region)
   :initform (cl:make-array 0 :element-type 'cooperative_driving_vision-msg:Region :initial-element (cl:make-instance 'cooperative_driving_vision-msg:Region)))
   (box_width
    :reader box_width
    :initarg :box_width
    :type cl:fixnum
    :initform 0)
   (box_height
    :reader box_height
    :initarg :box_height
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Features (<Features>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Features>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Features)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cooperative_driving_vision-msg:<Features> is deprecated: use cooperative_driving_vision-msg:Features instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Features>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_vision-msg:header-val is deprecated.  Use cooperative_driving_vision-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'image_width-val :lambda-list '(m))
(cl:defmethod image_width-val ((m <Features>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_vision-msg:image_width-val is deprecated.  Use cooperative_driving_vision-msg:image_width instead.")
  (image_width m))

(cl:ensure-generic-function 'image_height-val :lambda-list '(m))
(cl:defmethod image_height-val ((m <Features>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_vision-msg:image_height-val is deprecated.  Use cooperative_driving_vision-msg:image_height instead.")
  (image_height m))

(cl:ensure-generic-function 'Hlines-val :lambda-list '(m))
(cl:defmethod Hlines-val ((m <Features>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_vision-msg:Hlines-val is deprecated.  Use cooperative_driving_vision-msg:Hlines instead.")
  (Hlines m))

(cl:ensure-generic-function 'Vlines-val :lambda-list '(m))
(cl:defmethod Vlines-val ((m <Features>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_vision-msg:Vlines-val is deprecated.  Use cooperative_driving_vision-msg:Vlines instead.")
  (Vlines m))

(cl:ensure-generic-function 'PFPS-val :lambda-list '(m))
(cl:defmethod PFPS-val ((m <Features>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_vision-msg:PFPS-val is deprecated.  Use cooperative_driving_vision-msg:PFPS instead.")
  (PFPS m))

(cl:ensure-generic-function 'regions-val :lambda-list '(m))
(cl:defmethod regions-val ((m <Features>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_vision-msg:regions-val is deprecated.  Use cooperative_driving_vision-msg:regions instead.")
  (regions m))

(cl:ensure-generic-function 'box_width-val :lambda-list '(m))
(cl:defmethod box_width-val ((m <Features>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_vision-msg:box_width-val is deprecated.  Use cooperative_driving_vision-msg:box_width instead.")
  (box_width m))

(cl:ensure-generic-function 'box_height-val :lambda-list '(m))
(cl:defmethod box_height-val ((m <Features>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cooperative_driving_vision-msg:box_height-val is deprecated.  Use cooperative_driving_vision-msg:box_height instead.")
  (box_height m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Features>) ostream)
  "Serializes a message object of type '<Features>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'image_width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'image_width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'image_height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'image_height)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'Hlines))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'Hlines))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'Vlines))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'Vlines))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'PFPS)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'PFPS)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'regions))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'regions))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'box_width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'box_width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'box_height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'box_height)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Features>) istream)
  "Deserializes a message object of type '<Features>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'image_width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'image_width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'image_height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'image_height)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'Hlines) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'Hlines)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'Vlines) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'Vlines)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'PFPS)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'PFPS)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'regions) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'regions)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'cooperative_driving_vision-msg:Region))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'box_width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'box_width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'box_height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'box_height)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Features>)))
  "Returns string type for a message object of type '<Features>"
  "cooperative_driving_vision/Features")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Features)))
  "Returns string type for a message object of type 'Features"
  "cooperative_driving_vision/Features")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Features>)))
  "Returns md5sum for a message object of type '<Features>"
  "b06b1ffd262cf7d0705951b2a3148fc6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Features)))
  "Returns md5sum for a message object of type 'Features"
  "b06b1ffd262cf7d0705951b2a3148fc6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Features>)))
  "Returns full string definition for message of type '<Features>"
  (cl:format cl:nil "# Exchange format for extracted features~%~%# Standard ROS message header~%Header header~%# Width of the image the features where extracted from~%uint16 image_width~%# Height of the image the features where extracted from~%uint16 image_height~%# List of points in the image at which a vertical line~%# (the lane marker) has been extracted. The list is ordered~%# inversely by the y-coordinate, i.e. from bottom of the~%# image to the top.~%geometry_msgs/Point[] Hlines # z-coordinate unused~%geometry_msgs/Point[] Vlines~%# The processed frames per second~%uint16 PFPS~%# List of colored regions found in the image~%Region[] regions~%uint16 box_width~%uint16 box_height~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: cooperative_driving_vision/Region~%# Custom message type to represent an extracted region used for publishing~%~%# The region's average color~%std_msgs/ColorRGBA color~%# The geometrical moment describing the extents of the region~%Moment moment~%~%================================================================================~%MSG: std_msgs/ColorRGBA~%float32 r~%float32 g~%float32 b~%float32 a~%~%================================================================================~%MSG: cooperative_driving_vision/Moment~%# Custom message type to represent a geometrical moment~%~%# (0, 0)th raw moment, i.e. number of pixels~%float32 m00~%# (1, 0)th raw moment~%float32 m10~%# (0, 1)th raw moment~%float32 m01~%# (1, 1)th raw moment~%float32 m11~%# (2, 0)th raw moment~%float32 m20~%# (0, 2)th raw moment~%float32 m02~%# (2, 1)th raw moment~%float32 m21~%# (1, 2)th raw moment~%float32 m12~%# (3, 0)th raw moment~%float32 m30~%# (0, 3)th raw moment~%float32 m03~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Features)))
  "Returns full string definition for message of type 'Features"
  (cl:format cl:nil "# Exchange format for extracted features~%~%# Standard ROS message header~%Header header~%# Width of the image the features where extracted from~%uint16 image_width~%# Height of the image the features where extracted from~%uint16 image_height~%# List of points in the image at which a vertical line~%# (the lane marker) has been extracted. The list is ordered~%# inversely by the y-coordinate, i.e. from bottom of the~%# image to the top.~%geometry_msgs/Point[] Hlines # z-coordinate unused~%geometry_msgs/Point[] Vlines~%# The processed frames per second~%uint16 PFPS~%# List of colored regions found in the image~%Region[] regions~%uint16 box_width~%uint16 box_height~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: cooperative_driving_vision/Region~%# Custom message type to represent an extracted region used for publishing~%~%# The region's average color~%std_msgs/ColorRGBA color~%# The geometrical moment describing the extents of the region~%Moment moment~%~%================================================================================~%MSG: std_msgs/ColorRGBA~%float32 r~%float32 g~%float32 b~%float32 a~%~%================================================================================~%MSG: cooperative_driving_vision/Moment~%# Custom message type to represent a geometrical moment~%~%# (0, 0)th raw moment, i.e. number of pixels~%float32 m00~%# (1, 0)th raw moment~%float32 m10~%# (0, 1)th raw moment~%float32 m01~%# (1, 1)th raw moment~%float32 m11~%# (2, 0)th raw moment~%float32 m20~%# (0, 2)th raw moment~%float32 m02~%# (2, 1)th raw moment~%float32 m21~%# (1, 2)th raw moment~%float32 m12~%# (3, 0)th raw moment~%float32 m30~%# (0, 3)th raw moment~%float32 m03~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Features>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     2
     2
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'Hlines) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'Vlines) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     2
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'regions) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Features>))
  "Converts a ROS message object to a list"
  (cl:list 'Features
    (cl:cons ':header (header msg))
    (cl:cons ':image_width (image_width msg))
    (cl:cons ':image_height (image_height msg))
    (cl:cons ':Hlines (Hlines msg))
    (cl:cons ':Vlines (Vlines msg))
    (cl:cons ':PFPS (PFPS msg))
    (cl:cons ':regions (regions msg))
    (cl:cons ':box_width (box_width msg))
    (cl:cons ':box_height (box_height msg))
))
