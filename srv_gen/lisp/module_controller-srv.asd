
(cl:in-package :asdf)

(defsystem "module_controller-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "PollServoAngle" :depends-on ("_package_PollServoAngle"))
    (:file "_package_PollServoAngle" :depends-on ("_package"))
  ))