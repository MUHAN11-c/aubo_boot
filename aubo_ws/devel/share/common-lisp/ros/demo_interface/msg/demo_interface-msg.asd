
(cl:in-package :asdf)

(defsystem "demo_interface-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "RobotIOStatus" :depends-on ("_package_RobotIOStatus"))
    (:file "_package_RobotIOStatus" :depends-on ("_package"))
    (:file "RobotStatus" :depends-on ("_package_RobotStatus"))
    (:file "_package_RobotStatus" :depends-on ("_package"))
    (:file "ToolIOStatus" :depends-on ("_package_ToolIOStatus"))
    (:file "_package_ToolIOStatus" :depends-on ("_package"))
  ))