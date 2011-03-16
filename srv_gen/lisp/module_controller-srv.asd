
(cl:in-package :asdf)

(defsystem "module_controller-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "SetServoPower" :depends-on ("_package_SetServoPower"))
    (:file "_package_SetServoPower" :depends-on ("_package"))
    (:file "SetServoAngle" :depends-on ("_package_SetServoAngle"))
    (:file "_package_SetServoAngle" :depends-on ("_package"))
    (:file "GetServoAngle" :depends-on ("_package_GetServoAngle"))
    (:file "_package_GetServoAngle" :depends-on ("_package"))
  ))