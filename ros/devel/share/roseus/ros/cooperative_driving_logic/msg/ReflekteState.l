;; Auto-generated. Do not edit!


(when (boundp 'cooperative_driving_logic::ReflekteState)
  (if (not (find-package "COOPERATIVE_DRIVING_LOGIC"))
    (make-package "COOPERATIVE_DRIVING_LOGIC"))
  (shadow 'ReflekteState (find-package "COOPERATIVE_DRIVING_LOGIC")))
(unless (find-package "COOPERATIVE_DRIVING_LOGIC::REFLEKTESTATE")
  (make-package "COOPERATIVE_DRIVING_LOGIC::REFLEKTESTATE"))

(in-package "ROS")
;;//! \htmlinclude ReflekteState.msg.html


(defclass cooperative_driving_logic::ReflekteState
  :super ros::object
  :slots (_state _node_to_follow ))

(defmethod cooperative_driving_logic::ReflekteState
  (:init
   (&key
    ((:state __state) "")
    ((:node_to_follow __node_to_follow) 0)
    )
   (send-super :init)
   (setq _state (string __state))
   (setq _node_to_follow (round __node_to_follow))
   self)
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:node_to_follow
   (&optional __node_to_follow)
   (if __node_to_follow (setq _node_to_follow __node_to_follow)) _node_to_follow)
  (:serialization-length
   ()
   (+
    ;; string _state
    4 (length _state)
    ;; int8 _node_to_follow
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _state
       (write-long (length _state) s) (princ _state s)
     ;; int8 _node_to_follow
       (write-byte _node_to_follow s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _state
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _state (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int8 _node_to_follow
     (setq _node_to_follow (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _node_to_follow 127) (setq _node_to_follow (- _node_to_follow 256)))
   ;;
   self)
  )

(setf (get cooperative_driving_logic::ReflekteState :md5sum-) "43cc3c773585d3b826a620cbff7f3808")
(setf (get cooperative_driving_logic::ReflekteState :datatype-) "cooperative_driving_logic/ReflekteState")
(setf (get cooperative_driving_logic::ReflekteState :definition-)
      "# Used to publish the current state of the reactive behavior in the Reflekte node

# The current state of the reactive behavior
string state
int8 node_to_follow

")



(provide :cooperative_driving_logic/ReflekteState "43cc3c773585d3b826a620cbff7f3808")


