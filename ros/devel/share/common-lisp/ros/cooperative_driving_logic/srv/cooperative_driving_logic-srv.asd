
(cl:in-package :asdf)

(defsystem "cooperative_driving_logic-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ChangeState" :depends-on ("_package_ChangeState"))
    (:file "_package_ChangeState" :depends-on ("_package"))
  ))