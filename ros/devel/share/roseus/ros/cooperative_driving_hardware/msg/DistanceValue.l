;; Auto-generated. Do not edit!


(when (boundp 'cooperative_driving_hardware::DistanceValue)
  (if (not (find-package "COOPERATIVE_DRIVING_HARDWARE"))
    (make-package "COOPERATIVE_DRIVING_HARDWARE"))
  (shadow 'DistanceValue (find-package "COOPERATIVE_DRIVING_HARDWARE")))
(unless (find-package "COOPERATIVE_DRIVING_HARDWARE::DISTANCEVALUE")
  (make-package "COOPERATIVE_DRIVING_HARDWARE::DISTANCEVALUE"))

(in-package "ROS")
;;//! \htmlinclude DistanceValue.msg.html


(defclass cooperative_driving_hardware::DistanceValue
  :super ros::object
  :slots (_frame_id _distance ))

(defmethod cooperative_driving_hardware::DistanceValue
  (:init
   (&key
    ((:frame_id __frame_id) "")
    ((:distance __distance) 0.0)
    )
   (send-super :init)
   (setq _frame_id (string __frame_id))
   (setq _distance (float __distance))
   self)
  (:frame_id
   (&optional __frame_id)
   (if __frame_id (setq _frame_id __frame_id)) _frame_id)
  (:distance
   (&optional __distance)
   (if __distance (setq _distance __distance)) _distance)
  (:serialization-length
   ()
   (+
    ;; string _frame_id
    4 (length _frame_id)
    ;; float32 _distance
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _frame_id
       (write-long (length _frame_id) s) (princ _frame_id s)
     ;; float32 _distance
       (sys::poke _distance (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _frame_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _frame_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _distance
     (setq _distance (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get cooperative_driving_hardware::DistanceValue :md5sum-) "6464eba0f0ad6d8ca3357c316d6b0ed2")
(setf (get cooperative_driving_hardware::DistanceValue :datatype-) "cooperative_driving_hardware/DistanceValue")
(setf (get cooperative_driving_hardware::DistanceValue :definition-)
      "# Custom message type represent a reading from a distance sensor used for reading

# The frame_id (name) of this sensor
string frame_id
# The reading of this sensor
# Values in m
float32 distance

")



(provide :cooperative_driving_hardware/DistanceValue "6464eba0f0ad6d8ca3357c316d6b0ed2")


