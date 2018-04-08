;; Auto-generated. Do not edit!


(when (boundp 'cooperative_driving_msgs::Direction)
  (if (not (find-package "COOPERATIVE_DRIVING_MSGS"))
    (make-package "COOPERATIVE_DRIVING_MSGS"))
  (shadow 'Direction (find-package "COOPERATIVE_DRIVING_MSGS")))
(unless (find-package "COOPERATIVE_DRIVING_MSGS::DIRECTION")
  (make-package "COOPERATIVE_DRIVING_MSGS::DIRECTION"))

(in-package "ROS")
;;//! \htmlinclude Direction.msg.html


(defclass cooperative_driving_msgs::Direction
  :super ros::object
  :slots (_direction ))

(defmethod cooperative_driving_msgs::Direction
  (:init
   (&key
    ((:direction __direction) "")
    )
   (send-super :init)
   (setq _direction (string __direction))
   self)
  (:direction
   (&optional __direction)
   (if __direction (setq _direction __direction)) _direction)
  (:serialization-length
   ()
   (+
    ;; string _direction
    4 (length _direction)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _direction
       (write-long (length _direction) s) (princ _direction s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _direction
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _direction (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get cooperative_driving_msgs::Direction :md5sum-) "326e9417def5db9a05a2704a4d8de15e")
(setf (get cooperative_driving_msgs::Direction :datatype-) "cooperative_driving_msgs/Direction")
(setf (get cooperative_driving_msgs::Direction :definition-)
      "## Define a direction of a detected crossing, i.e., a turn that can possibly be taken

# Possible turn at a crossing ('straight', 'left', or 'right')
string direction
")



(provide :cooperative_driving_msgs/Direction "326e9417def5db9a05a2704a4d8de15e")


