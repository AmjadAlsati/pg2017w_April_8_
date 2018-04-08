;; Auto-generated. Do not edit!


(when (boundp 'cooperative_driving_logic::ChangeState)
  (if (not (find-package "COOPERATIVE_DRIVING_LOGIC"))
    (make-package "COOPERATIVE_DRIVING_LOGIC"))
  (shadow 'ChangeState (find-package "COOPERATIVE_DRIVING_LOGIC")))
(unless (find-package "COOPERATIVE_DRIVING_LOGIC::CHANGESTATE")
  (make-package "COOPERATIVE_DRIVING_LOGIC::CHANGESTATE"))
(unless (find-package "COOPERATIVE_DRIVING_LOGIC::CHANGESTATEREQUEST")
  (make-package "COOPERATIVE_DRIVING_LOGIC::CHANGESTATEREQUEST"))
(unless (find-package "COOPERATIVE_DRIVING_LOGIC::CHANGESTATERESPONSE")
  (make-package "COOPERATIVE_DRIVING_LOGIC::CHANGESTATERESPONSE"))

(in-package "ROS")





(intern "*REMOTE_CONTROL*" (find-package "COOPERATIVE_DRIVING_LOGIC::CHANGESTATEREQUEST"))
(shadow '*REMOTE_CONTROL* (find-package "COOPERATIVE_DRIVING_LOGIC::CHANGESTATEREQUEST"))
(defconstant cooperative_driving_logic::ChangeStateRequest::*REMOTE_CONTROL* "remote_control")
(intern "*FOLLOW_BLOB*" (find-package "COOPERATIVE_DRIVING_LOGIC::CHANGESTATEREQUEST"))
(shadow '*FOLLOW_BLOB* (find-package "COOPERATIVE_DRIVING_LOGIC::CHANGESTATEREQUEST"))
(defconstant cooperative_driving_logic::ChangeStateRequest::*FOLLOW_BLOB* "follow_blob")
(intern "*FOLLOW_LINE*" (find-package "COOPERATIVE_DRIVING_LOGIC::CHANGESTATEREQUEST"))
(shadow '*FOLLOW_LINE* (find-package "COOPERATIVE_DRIVING_LOGIC::CHANGESTATEREQUEST"))
(defconstant cooperative_driving_logic::ChangeStateRequest::*FOLLOW_LINE* "follow_line")
(intern "*PLATOONING*" (find-package "COOPERATIVE_DRIVING_LOGIC::CHANGESTATEREQUEST"))
(shadow '*PLATOONING* (find-package "COOPERATIVE_DRIVING_LOGIC::CHANGESTATEREQUEST"))
(defconstant cooperative_driving_logic::ChangeStateRequest::*PLATOONING* "platooning")
(intern "*DYNAMIC_FOLLOW_LINE*" (find-package "COOPERATIVE_DRIVING_LOGIC::CHANGESTATEREQUEST"))
(shadow '*DYNAMIC_FOLLOW_LINE* (find-package "COOPERATIVE_DRIVING_LOGIC::CHANGESTATEREQUEST"))
(defconstant cooperative_driving_logic::ChangeStateRequest::*DYNAMIC_FOLLOW_LINE* "dynamic_follow_line")
(intern "*IDLE*" (find-package "COOPERATIVE_DRIVING_LOGIC::CHANGESTATEREQUEST"))
(shadow '*IDLE* (find-package "COOPERATIVE_DRIVING_LOGIC::CHANGESTATEREQUEST"))
(defconstant cooperative_driving_logic::ChangeStateRequest::*IDLE* "idle")
(defclass cooperative_driving_logic::ChangeStateRequest
  :super ros::object
  :slots (_target_state _tag_id_to_follow ))

(defmethod cooperative_driving_logic::ChangeStateRequest
  (:init
   (&key
    ((:target_state __target_state) "")
    ((:tag_id_to_follow __tag_id_to_follow) 0)
    )
   (send-super :init)
   (setq _target_state (string __target_state))
   (setq _tag_id_to_follow (round __tag_id_to_follow))
   self)
  (:target_state
   (&optional __target_state)
   (if __target_state (setq _target_state __target_state)) _target_state)
  (:tag_id_to_follow
   (&optional __tag_id_to_follow)
   (if __tag_id_to_follow (setq _tag_id_to_follow __tag_id_to_follow)) _tag_id_to_follow)
  (:serialization-length
   ()
   (+
    ;; string _target_state
    4 (length _target_state)
    ;; int8 _tag_id_to_follow
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _target_state
       (write-long (length _target_state) s) (princ _target_state s)
     ;; int8 _tag_id_to_follow
       (write-byte _tag_id_to_follow s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _target_state
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _target_state (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int8 _tag_id_to_follow
     (setq _tag_id_to_follow (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _tag_id_to_follow 127) (setq _tag_id_to_follow (- _tag_id_to_follow 256)))
   ;;
   self)
  )

(defclass cooperative_driving_logic::ChangeStateResponse
  :super ros::object
  :slots ())

(defmethod cooperative_driving_logic::ChangeStateResponse
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

(defclass cooperative_driving_logic::ChangeState
  :super ros::object
  :slots ())

(setf (get cooperative_driving_logic::ChangeState :md5sum-) "47b07bdab417ccb3c955c49a6d18057b")
(setf (get cooperative_driving_logic::ChangeState :datatype-) "cooperative_driving_logic/ChangeState")
(setf (get cooperative_driving_logic::ChangeState :request) cooperative_driving_logic::ChangeStateRequest)
(setf (get cooperative_driving_logic::ChangeState :response) cooperative_driving_logic::ChangeStateResponse)

(defmethod cooperative_driving_logic::ChangeStateRequest
  (:response () (instance cooperative_driving_logic::ChangeStateResponse :init)))

(setf (get cooperative_driving_logic::ChangeStateRequest :md5sum-) "47b07bdab417ccb3c955c49a6d18057b")
(setf (get cooperative_driving_logic::ChangeStateRequest :datatype-) "cooperative_driving_logic/ChangeStateRequest")
(setf (get cooperative_driving_logic::ChangeStateRequest :definition-)
      "
string REMOTE_CONTROL=remote_control
string FOLLOW_BLOB=follow_blob
string FOLLOW_LINE=follow_line
string PLATOONING=platooning
string DYNAMIC_FOLLOW_LINE=dynamic_follow_line
string IDLE=idle

string target_state
int8 tag_id_to_follow
---

")

(setf (get cooperative_driving_logic::ChangeStateResponse :md5sum-) "47b07bdab417ccb3c955c49a6d18057b")
(setf (get cooperative_driving_logic::ChangeStateResponse :datatype-) "cooperative_driving_logic/ChangeStateResponse")
(setf (get cooperative_driving_logic::ChangeStateResponse :definition-)
      "
string REMOTE_CONTROL=remote_control
string FOLLOW_BLOB=follow_blob
string FOLLOW_LINE=follow_line
string PLATOONING=platooning
string DYNAMIC_FOLLOW_LINE=dynamic_follow_line
string IDLE=idle

string target_state
int8 tag_id_to_follow
---

")



(provide :cooperative_driving_logic/ChangeState "47b07bdab417ccb3c955c49a6d18057b")


