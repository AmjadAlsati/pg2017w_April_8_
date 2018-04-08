;; Auto-generated. Do not edit!


(when (boundp 'cooperative_driving_hardware::LedValue)
  (if (not (find-package "COOPERATIVE_DRIVING_HARDWARE"))
    (make-package "COOPERATIVE_DRIVING_HARDWARE"))
  (shadow 'LedValue (find-package "COOPERATIVE_DRIVING_HARDWARE")))
(unless (find-package "COOPERATIVE_DRIVING_HARDWARE::LEDVALUE")
  (make-package "COOPERATIVE_DRIVING_HARDWARE::LEDVALUE"))

(in-package "ROS")
;;//! \htmlinclude LedValue.msg.html


(defclass cooperative_driving_hardware::LedValue
  :super ros::object
  :slots (_frame_id _value ))

(defmethod cooperative_driving_hardware::LedValue
  (:init
   (&key
    ((:frame_id __frame_id) "")
    ((:value __value) 0.0)
    )
   (send-super :init)
   (setq _frame_id (string __frame_id))
   (setq _value (float __value))
   self)
  (:frame_id
   (&optional __frame_id)
   (if __frame_id (setq _frame_id __frame_id)) _frame_id)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:serialization-length
   ()
   (+
    ;; string _frame_id
    4 (length _frame_id)
    ;; float32 _value
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _frame_id
       (write-long (length _frame_id) s) (princ _frame_id s)
     ;; float32 _value
       (sys::poke _value (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _frame_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _frame_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _value
     (setq _value (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get cooperative_driving_hardware::LedValue :md5sum-) "6a0c35f631f3f53c7b19b24b9d87c563")
(setf (get cooperative_driving_hardware::LedValue :datatype-) "cooperative_driving_hardware/LedValue")
(setf (get cooperative_driving_hardware::LedValue :definition-)
      "# Custom message type represent a reading from a distance sensor used for reading and writing

# The frame_id (name) of this LED
string frame_id
# The current value of this LED
# Values in [0, 1]
float32 value

")



(provide :cooperative_driving_hardware/LedValue "6a0c35f631f3f53c7b19b24b9d87c563")


