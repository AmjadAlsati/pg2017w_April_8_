;; Auto-generated. Do not edit!


(when (boundp 'cooperative_driving_networking::BroadcastMessage)
  (if (not (find-package "COOPERATIVE_DRIVING_NETWORKING"))
    (make-package "COOPERATIVE_DRIVING_NETWORKING"))
  (shadow 'BroadcastMessage (find-package "COOPERATIVE_DRIVING_NETWORKING")))
(unless (find-package "COOPERATIVE_DRIVING_NETWORKING::BROADCASTMESSAGE")
  (make-package "COOPERATIVE_DRIVING_NETWORKING::BROADCASTMESSAGE"))
(unless (find-package "COOPERATIVE_DRIVING_NETWORKING::BROADCASTMESSAGEREQUEST")
  (make-package "COOPERATIVE_DRIVING_NETWORKING::BROADCASTMESSAGEREQUEST"))
(unless (find-package "COOPERATIVE_DRIVING_NETWORKING::BROADCASTMESSAGERESPONSE")
  (make-package "COOPERATIVE_DRIVING_NETWORKING::BROADCASTMESSAGERESPONSE"))

(in-package "ROS")





(defclass cooperative_driving_networking::BroadcastMessageRequest
  :super ros::object
  :slots (_msg ))

(defmethod cooperative_driving_networking::BroadcastMessageRequest
  (:init
   (&key
    ((:msg __msg) "")
    )
   (send-super :init)
   (setq _msg (string __msg))
   self)
  (:msg
   (&optional __msg)
   (if __msg (setq _msg __msg)) _msg)
  (:serialization-length
   ()
   (+
    ;; string _msg
    4 (length _msg)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _msg
       (write-long (length _msg) s) (princ _msg s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _msg
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _msg (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass cooperative_driving_networking::BroadcastMessageResponse
  :super ros::object
  :slots ())

(defmethod cooperative_driving_networking::BroadcastMessageResponse
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass cooperative_driving_networking::BroadcastMessage
  :super ros::object
  :slots ())

(setf (get cooperative_driving_networking::BroadcastMessage :md5sum-) "7d96ed730776804754140b85e64c862e")
(setf (get cooperative_driving_networking::BroadcastMessage :datatype-) "cooperative_driving_networking/BroadcastMessage")
(setf (get cooperative_driving_networking::BroadcastMessage :request) cooperative_driving_networking::BroadcastMessageRequest)
(setf (get cooperative_driving_networking::BroadcastMessage :response) cooperative_driving_networking::BroadcastMessageResponse)

(defmethod cooperative_driving_networking::BroadcastMessageRequest
  (:response () (instance cooperative_driving_networking::BroadcastMessageResponse :init)))

(setf (get cooperative_driving_networking::BroadcastMessageRequest :md5sum-) "7d96ed730776804754140b85e64c862e")
(setf (get cooperative_driving_networking::BroadcastMessageRequest :datatype-) "cooperative_driving_networking/BroadcastMessageRequest")
(setf (get cooperative_driving_networking::BroadcastMessageRequest :definition-)
      "string msg
---
")

(setf (get cooperative_driving_networking::BroadcastMessageResponse :md5sum-) "7d96ed730776804754140b85e64c862e")
(setf (get cooperative_driving_networking::BroadcastMessageResponse :datatype-) "cooperative_driving_networking/BroadcastMessageResponse")
(setf (get cooperative_driving_networking::BroadcastMessageResponse :definition-)
      "string msg
---
")



(provide :cooperative_driving_networking/BroadcastMessage "7d96ed730776804754140b85e64c862e")


