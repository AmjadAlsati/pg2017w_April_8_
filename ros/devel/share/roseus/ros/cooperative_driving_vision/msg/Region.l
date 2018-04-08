;; Auto-generated. Do not edit!


(when (boundp 'cooperative_driving_vision::Region)
  (if (not (find-package "COOPERATIVE_DRIVING_VISION"))
    (make-package "COOPERATIVE_DRIVING_VISION"))
  (shadow 'Region (find-package "COOPERATIVE_DRIVING_VISION")))
(unless (find-package "COOPERATIVE_DRIVING_VISION::REGION")
  (make-package "COOPERATIVE_DRIVING_VISION::REGION"))

(in-package "ROS")
;;//! \htmlinclude Region.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass cooperative_driving_vision::Region
  :super ros::object
  :slots (_color _moment ))

(defmethod cooperative_driving_vision::Region
  (:init
   (&key
    ((:color __color) (instance std_msgs::ColorRGBA :init))
    ((:moment __moment) (instance cooperative_driving_vision::Moment :init))
    )
   (send-super :init)
   (setq _color __color)
   (setq _moment __moment)
   self)
  (:color
   (&rest __color)
   (if (keywordp (car __color))
       (send* _color __color)
     (progn
       (if __color (setq _color (car __color)))
       _color)))
  (:moment
   (&rest __moment)
   (if (keywordp (car __moment))
       (send* _moment __moment)
     (progn
       (if __moment (setq _moment (car __moment)))
       _moment)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/ColorRGBA _color
    (send _color :serialization-length)
    ;; cooperative_driving_vision/Moment _moment
    (send _moment :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/ColorRGBA _color
       (send _color :serialize s)
     ;; cooperative_driving_vision/Moment _moment
       (send _moment :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/ColorRGBA _color
     (send _color :deserialize buf ptr-) (incf ptr- (send _color :serialization-length))
   ;; cooperative_driving_vision/Moment _moment
     (send _moment :deserialize buf ptr-) (incf ptr- (send _moment :serialization-length))
   ;;
   self)
  )

(setf (get cooperative_driving_vision::Region :md5sum-) "b981b501a8a1ad1a59b87231a82a1888")
(setf (get cooperative_driving_vision::Region :datatype-) "cooperative_driving_vision/Region")
(setf (get cooperative_driving_vision::Region :definition-)
      "# Custom message type to represent an extracted region used for publishing

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



(provide :cooperative_driving_vision/Region "b981b501a8a1ad1a59b87231a82a1888")


