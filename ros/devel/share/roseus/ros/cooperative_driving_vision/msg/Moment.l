;; Auto-generated. Do not edit!


(when (boundp 'cooperative_driving_vision::Moment)
  (if (not (find-package "COOPERATIVE_DRIVING_VISION"))
    (make-package "COOPERATIVE_DRIVING_VISION"))
  (shadow 'Moment (find-package "COOPERATIVE_DRIVING_VISION")))
(unless (find-package "COOPERATIVE_DRIVING_VISION::MOMENT")
  (make-package "COOPERATIVE_DRIVING_VISION::MOMENT"))

(in-package "ROS")
;;//! \htmlinclude Moment.msg.html


(defclass cooperative_driving_vision::Moment
  :super ros::object
  :slots (_m00 _m10 _m01 _m11 _m20 _m02 _m21 _m12 _m30 _m03 ))

(defmethod cooperative_driving_vision::Moment
  (:init
   (&key
    ((:m00 __m00) 0.0)
    ((:m10 __m10) 0.0)
    ((:m01 __m01) 0.0)
    ((:m11 __m11) 0.0)
    ((:m20 __m20) 0.0)
    ((:m02 __m02) 0.0)
    ((:m21 __m21) 0.0)
    ((:m12 __m12) 0.0)
    ((:m30 __m30) 0.0)
    ((:m03 __m03) 0.0)
    )
   (send-super :init)
   (setq _m00 (float __m00))
   (setq _m10 (float __m10))
   (setq _m01 (float __m01))
   (setq _m11 (float __m11))
   (setq _m20 (float __m20))
   (setq _m02 (float __m02))
   (setq _m21 (float __m21))
   (setq _m12 (float __m12))
   (setq _m30 (float __m30))
   (setq _m03 (float __m03))
   self)
  (:m00
   (&optional __m00)
   (if __m00 (setq _m00 __m00)) _m00)
  (:m10
   (&optional __m10)
   (if __m10 (setq _m10 __m10)) _m10)
  (:m01
   (&optional __m01)
   (if __m01 (setq _m01 __m01)) _m01)
  (:m11
   (&optional __m11)
   (if __m11 (setq _m11 __m11)) _m11)
  (:m20
   (&optional __m20)
   (if __m20 (setq _m20 __m20)) _m20)
  (:m02
   (&optional __m02)
   (if __m02 (setq _m02 __m02)) _m02)
  (:m21
   (&optional __m21)
   (if __m21 (setq _m21 __m21)) _m21)
  (:m12
   (&optional __m12)
   (if __m12 (setq _m12 __m12)) _m12)
  (:m30
   (&optional __m30)
   (if __m30 (setq _m30 __m30)) _m30)
  (:m03
   (&optional __m03)
   (if __m03 (setq _m03 __m03)) _m03)
  (:serialization-length
   ()
   (+
    ;; float32 _m00
    4
    ;; float32 _m10
    4
    ;; float32 _m01
    4
    ;; float32 _m11
    4
    ;; float32 _m20
    4
    ;; float32 _m02
    4
    ;; float32 _m21
    4
    ;; float32 _m12
    4
    ;; float32 _m30
    4
    ;; float32 _m03
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _m00
       (sys::poke _m00 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _m10
       (sys::poke _m10 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _m01
       (sys::poke _m01 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _m11
       (sys::poke _m11 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _m20
       (sys::poke _m20 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _m02
       (sys::poke _m02 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _m21
       (sys::poke _m21 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _m12
       (sys::poke _m12 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _m30
       (sys::poke _m30 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _m03
       (sys::poke _m03 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _m00
     (setq _m00 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _m10
     (setq _m10 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _m01
     (setq _m01 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _m11
     (setq _m11 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _m20
     (setq _m20 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _m02
     (setq _m02 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _m21
     (setq _m21 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _m12
     (setq _m12 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _m30
     (setq _m30 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _m03
     (setq _m03 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get cooperative_driving_vision::Moment :md5sum-) "15ca49fd130f761e715cfaf6f1985ad7")
(setf (get cooperative_driving_vision::Moment :datatype-) "cooperative_driving_vision/Moment")
(setf (get cooperative_driving_vision::Moment :definition-)
      "# Custom message type to represent a geometrical moment

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



(provide :cooperative_driving_vision/Moment "15ca49fd130f761e715cfaf6f1985ad7")


