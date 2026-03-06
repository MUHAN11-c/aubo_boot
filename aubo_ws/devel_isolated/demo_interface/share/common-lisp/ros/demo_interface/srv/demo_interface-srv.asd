
(cl:in-package :asdf)

(defsystem "demo_interface-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :trajectory_msgs-msg
)
  :components ((:file "_package")
    (:file "ExecuteTrajectory" :depends-on ("_package_ExecuteTrajectory"))
    (:file "_package_ExecuteTrajectory" :depends-on ("_package"))
    (:file "GetCurrentState" :depends-on ("_package_GetCurrentState"))
    (:file "_package_GetCurrentState" :depends-on ("_package"))
    (:file "MoveToPose" :depends-on ("_package_MoveToPose"))
    (:file "_package_MoveToPose" :depends-on ("_package"))
    (:file "PlanTrajectory" :depends-on ("_package_PlanTrajectory"))
    (:file "_package_PlanTrajectory" :depends-on ("_package"))
    (:file "ReadRobotIO" :depends-on ("_package_ReadRobotIO"))
    (:file "_package_ReadRobotIO" :depends-on ("_package"))
    (:file "SetRobotEnable" :depends-on ("_package_SetRobotEnable"))
    (:file "_package_SetRobotEnable" :depends-on ("_package"))
    (:file "SetRobotIO" :depends-on ("_package_SetRobotIO"))
    (:file "_package_SetRobotIO" :depends-on ("_package"))
    (:file "SetSpeedFactor" :depends-on ("_package_SetSpeedFactor"))
    (:file "_package_SetSpeedFactor" :depends-on ("_package"))
  ))