;; Auto-generated. Do not edit!


(when (boundp 'cooperative_driving_msgs::Directions)
  (if (not (find-package "COOPERATIVE_DRIVING_MSGS"))
    (make-package "COOPERATIVE_DRIVING_MSGS"))
  (shadow 'Directions (find-package "COOPERATIVE_DRIVING_MSGS")))
(unless (find-package "COOPERATIVE_DRIVING_MSGS::DIRECTIONS")
  (make-package "COOPERATIVE_DRIVING_MSGS::DIRECTIONS"))

(in-package "ROS")
;;//! \htmlinclude Directions.msg.html


(defclass cooperative_driving_msgs::Directions
  :super ros::object
  :slots (_directions ))

(defmethod cooperative_driving_msgs::Directions
  (:init
   (&key
    ((:directions __directions) (let (r) (dotimes (i 0) (push (instance cooperative_driving_msgs::Direction :init) r)) r))
    )
   (send-super :init)
   (setq _directions __directions)
   self)
  (:directions
   (&rest __directions)
   (if (keywordp (car __directions))
       (send* _directions __directions)
     (progn
       (if __directions (setq _directions (car __directions)))
       _directions)))
  (:serialization-length
   ()
   (+
    ;; cooperative_driving_msgs/Direction[] _directions
    (apply #'+ (send-all _directions :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; cooperative_driving_msgs/Direction[] _directions
     (write-long (length _directions) s)
     (dolist (elem _directions)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; cooperative_driving_msgs/Direction[] _directions
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _directions (let (r) (dotimes (i n) (push (instance cooperative_driving_msgs::Direction :init) r)) r))
     (dolist (elem- _directions)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get cooperative_driving_msgs::Directions :md5sum-) "f467a95e7546b9935ac1d69c0341e385")
(setf (get cooperative_driving_msgs::Directions :datatype-) "cooperative_driving_msgs/Directions")
(setf (get cooperative_driving_msgs::Directions :definition-)
      "## List all directions of a detected crossing

# Array of all directions of a crossing 
Direction[] directions
================================================================================
MSG: cooperative_driving_msgs/Direction
## Define a direction of a detected crossing, i.e., a turn that can possibly be taken

# Possible turn at a crossing ('straight', 'left', or 'right')
string direction
")



(provide :cooperative_driving_msgs/Directions "f467a95e7546b9935ac1d69c0341e385")


