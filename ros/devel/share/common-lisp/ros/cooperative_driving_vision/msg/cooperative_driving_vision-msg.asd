
(cl:in-package :asdf)

(defsystem "cooperative_driving_vision-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Features" :depends-on ("_package_Features"))
    (:file "_package_Features" :depends-on ("_package"))
    (:file "Moment" :depends-on ("_package_Moment"))
    (:file "_package_Moment" :depends-on ("_package"))
    (:file "Region" :depends-on ("_package_Region"))
    (:file "_package_Region" :depends-on ("_package"))
  ))