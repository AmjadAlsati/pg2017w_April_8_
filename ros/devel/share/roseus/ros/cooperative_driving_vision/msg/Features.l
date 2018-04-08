;; Auto-generated. Do not edit!


(when (boundp 'cooperative_driving_vision::Features)
  (if (not (find-package "COOPERATIVE_DRIVING_VISION"))
    (make-package "COOPERATIVE_DRIVING_VISION"))
  (shadow 'Features (find-package "COOPERATIVE_DRIVING_VISION")))
(unless (find-package "COOPERATIVE_DRIVING_VISION::FEATURES")
  (make-package "COOPERATIVE_DRIVING_VISION::FEATURES"))

(in-package "ROS")
;;//! \htmlinclude Features.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass cooperative_driving_vision::Features
  :super ros::object
  :slots (_header _image_width _image_height _Hlines _Vlines _PFPS _regions _box_width _box_height ))

(defmethod cooperative_driving_vision::Features
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:image_width __image_width) 0)
    ((:image_height __image_height) 0)
    ((:Hlines __Hlines) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Point :init) r)) r))
    ((:Vlines __Vlines) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Point :init) r)) r))
    ((:PFPS __PFPS) 0)
    ((:regions __regions) (let (r) (dotimes (i 0) (push (instance cooperative_driving_vision::Region :init) r)) r))
    ((:box_width __box_width) 0)
    ((:box_height __box_height) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _image_width (round __image_width))
   (setq _image_height (round __image_height))
   (setq _Hlines __Hlines)
   (setq _Vlines __Vlines)
   (setq _PFPS (round __PFPS))
   (setq _regions __regions)
   (setq _box_width (round __box_width))
   (setq _box_height (round __box_height))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:image_width
   (&optional __image_width)
   (if __image_width (setq _image_width __image_width)) _image_width)
  (:image_height
   (&optional __image_height)
   (if __image_height (setq _image_height __image_height)) _image_height)
  (:Hlines
   (&rest __Hlines)
   (if (keywordp (car __Hlines))
       (send* _Hlines __Hlines)
     (progn
       (if __Hlines (setq _Hlines (car __Hlines)))
       _Hlines)))
  (:Vlines
   (&rest __Vlines)
   (if (keywordp (car __Vlines))
       (send* _Vlines __Vlines)
     (progn
       (if __Vlines (setq _Vlines (car __Vlines)))
       _Vlines)))
  (:PFPS
   (&optional __PFPS)
   (if __PFPS (setq _PFPS __PFPS)) _PFPS)
  (:regions
   (&rest __regions)
   (if (keywordp (car __regions))
       (send* _regions __regions)
     (progn
       (if __regions (setq _regions (car __regions)))
       _regions)))
  (:box_width
   (&optional __box_width)
   (if __box_width (setq _box_width __box_width)) _box_width)
  (:box_height
   (&optional __box_height)
   (if __box_height (setq _box_height __box_height)) _box_height)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint16 _image_width
    2
    ;; uint16 _image_height
    2
    ;; geometry_msgs/Point[] _Hlines
    (apply #'+ (send-all _Hlines :serialization-length)) 4
    ;; geometry_msgs/Point[] _Vlines
    (apply #'+ (send-all _Vlines :serialization-length)) 4
    ;; uint16 _PFPS
    2
    ;; cooperative_driving_vision/Region[] _regions
    (apply #'+ (send-all _regions :serialization-length)) 4
    ;; uint16 _box_width
    2
    ;; uint16 _box_height
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint16 _image_width
       (write-word _image_width s)
     ;; uint16 _image_height
       (write-word _image_height s)
     ;; geometry_msgs/Point[] _Hlines
     (write-long (length _Hlines) s)
     (dolist (elem _Hlines)
       (send elem :serialize s)
       )
     ;; geometry_msgs/Point[] _Vlines
     (write-long (length _Vlines) s)
     (dolist (elem _Vlines)
       (send elem :serialize s)
       )
     ;; uint16 _PFPS
       (write-word _PFPS s)
     ;; cooperative_driving_vision/Region[] _regions
     (write-long (length _regions) s)
     (dolist (elem _regions)
       (send elem :serialize s)
       )
     ;; uint16 _box_width
       (write-word _box_width s)
     ;; uint16 _box_height
       (write-word _box_height s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint16 _image_width
     (setq _image_width (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _image_height
     (setq _image_height (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; geometry_msgs/Point[] _Hlines
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _Hlines (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _Hlines)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/Point[] _Vlines
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _Vlines (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _Vlines)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; uint16 _PFPS
     (setq _PFPS (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; cooperative_driving_vision/Region[] _regions
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _regions (let (r) (dotimes (i n) (push (instance cooperative_driving_vision::Region :init) r)) r))
     (dolist (elem- _regions)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; uint16 _box_width
     (setq _box_width (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _box_height
     (setq _box_height (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get cooperative_driving_vision::Features :md5sum-) "b06b1ffd262cf7d0705951b2a3148fc6")
(setf (get cooperative_driving_vision::Features :datatype-) "cooperative_driving_vision/Features")
(setf (get cooperative_driving_vision::Features :definition-)
      "# Exchange format for extracted features

# Standard ROS message header
Header header
# Width of the image the features where extracted from
uint16 image_width
# Height of the image the features where extracted from
uint16 image_height
# List of points in the image at which a vertical line
# (the lane marker) has been extracted. The list is ordered
# inversely by the y-coordinate, i.e. from bottom of the
# image to the top.
geometry_msgs/Point[] Hlines # z-coordinate unused
geometry_msgs/Point[] Vlines
# The processed frames per second
uint16 PFPS
# List of colored regions found in the image
Region[] regions
uint16 box_width
uint16 box_height
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: cooperative_driving_vision/Region
# Custom message type to represent an extracted region used for publishing

# The region's average color
std_msgs/ColorRGBA color
# The geometrical moment describing the extents of the region
Moment moment

================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a

================================================================================
MSG: cooperative_driving_vision/Moment
# Custom message type to represent a geometrical moment

# (0, 0)th raw moment, i.e. number of pixels
float32 m00
# (1, 0)th raw moment
float32 m10
# (0, 1)th raw moment
float32 m01
# (1, 1)th raw moment
float32 m11
# (2, 0)th raw moment
float32 m20
# (0, 2)th raw moment
float32 m02
# (2, 1)th raw moment
float32 m21
# (1, 2)th raw moment
float32 m12
# (3, 0)th raw moment
float32 m30
# (0, 3)th raw moment
float32 m03

")



(provide :cooperative_driving_vision/Features "b06b1ffd262cf7d0705951b2a3148fc6")


