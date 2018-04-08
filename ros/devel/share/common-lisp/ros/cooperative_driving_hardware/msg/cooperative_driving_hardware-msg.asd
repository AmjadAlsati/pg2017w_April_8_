
(cl:in-package :asdf)

(defsystem "cooperative_driving_hardware-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "DistanceValue" :depends-on ("_package_DistanceValue"))
    (:file "_package_DistanceValue" :depends-on ("_package"))
    (:file "DistanceValues" :depends-on ("_package_DistanceValues"))
    (:file "_package_DistanceValues" :depends-on ("_package"))
    (:file "LedCommand" :depends-on ("_package_LedCommand"))
    (:file "_package_LedCommand" :depends-on ("_package"))
    (:file "LedValue" :depends-on ("_package_LedValue"))
    (:file "_package_LedValue" :depends-on ("_package"))
    (:file "LedValues" :depends-on ("_package_LedValues"))
    (:file "_package_LedValues" :depends-on ("_package"))
  ))