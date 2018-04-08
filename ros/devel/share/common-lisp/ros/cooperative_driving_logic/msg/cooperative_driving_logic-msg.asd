
(cl:in-package :asdf)

(defsystem "cooperative_driving_logic-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ReflekteState" :depends-on ("_package_ReflekteState"))
    (:file "_package_ReflekteState" :depends-on ("_package"))
  ))