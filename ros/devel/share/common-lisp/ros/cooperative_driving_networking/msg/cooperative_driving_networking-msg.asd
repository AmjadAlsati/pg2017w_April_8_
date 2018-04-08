
(cl:in-package :asdf)

(defsystem "cooperative_driving_networking-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "CooperativeAwarenessMessage" :depends-on ("_package_CooperativeAwarenessMessage"))
    (:file "_package_CooperativeAwarenessMessage" :depends-on ("_package"))
    (:file "EmergencyBrake" :depends-on ("_package_EmergencyBrake"))
    (:file "_package_EmergencyBrake" :depends-on ("_package"))
    (:file "Platooning" :depends-on ("_package_Platooning"))
    (:file "_package_Platooning" :depends-on ("_package"))
    (:file "StringStamped" :depends-on ("_package_StringStamped"))
    (:file "_package_StringStamped" :depends-on ("_package"))
    (:file "Token" :depends-on ("_package_Token"))
    (:file "_package_Token" :depends-on ("_package"))
  ))