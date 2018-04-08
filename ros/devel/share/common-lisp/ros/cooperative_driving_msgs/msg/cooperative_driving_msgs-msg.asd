
(cl:in-package :asdf)

(defsystem "cooperative_driving_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Command" :depends-on ("_package_Command"))
    (:file "_package_Command" :depends-on ("_package"))
    (:file "Direction" :depends-on ("_package_Direction"))
    (:file "_package_Direction" :depends-on ("_package"))
    (:file "Directions" :depends-on ("_package_Directions"))
    (:file "_package_Directions" :depends-on ("_package"))
  ))