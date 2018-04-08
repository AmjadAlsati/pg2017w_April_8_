
(cl:in-package :asdf)

(defsystem "cooperative_driving_networking-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "BroadcastMessage" :depends-on ("_package_BroadcastMessage"))
    (:file "_package_BroadcastMessage" :depends-on ("_package"))
  ))