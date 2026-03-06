
(cl:in-package :asdf)

(defsystem "aubo_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
               :trajectory_msgs-msg
)
  :components ((:file "_package")
    (:file "Analog" :depends-on ("_package_Analog"))
    (:file "_package_Analog" :depends-on ("_package"))
    (:file "Digital" :depends-on ("_package_Digital"))
    (:file "_package_Digital" :depends-on ("_package"))
    (:file "GoalPoint" :depends-on ("_package_GoalPoint"))
    (:file "_package_GoalPoint" :depends-on ("_package"))
    (:file "IOState" :depends-on ("_package_IOState"))
    (:file "_package_IOState" :depends-on ("_package"))
    (:file "JointPos" :depends-on ("_package_JointPos"))
    (:file "_package_JointPos" :depends-on ("_package"))
    (:file "JointTrajectoryFeedback" :depends-on ("_package_JointTrajectoryFeedback"))
    (:file "_package_JointTrajectoryFeedback" :depends-on ("_package"))
    (:file "TraPoint" :depends-on ("_package_TraPoint"))
    (:file "_package_TraPoint" :depends-on ("_package"))
  ))