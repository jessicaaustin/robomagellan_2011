
(in-package :asdf)

(defsystem "robomagellan-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :roslib-msg
)
  :components ((:file "_package")
    (:file "Move" :depends-on ("_package"))
    (:file "_package_Move" :depends-on ("_package"))
    (:file "Range" :depends-on ("_package"))
    (:file "_package_Range" :depends-on ("_package"))
    (:file "Gyro" :depends-on ("_package"))
    (:file "_package_Gyro" :depends-on ("_package"))
    ))
