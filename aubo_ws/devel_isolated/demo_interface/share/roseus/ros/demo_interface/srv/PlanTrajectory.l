;; Auto-generated. Do not edit!


(when (boundp 'demo_interface::PlanTrajectory)
  (if (not (find-package "DEMO_INTERFACE"))
    (make-package "DEMO_INTERFACE"))
  (shadow 'PlanTrajectory (find-package "DEMO_INTERFACE")))
(unless (find-package "DEMO_INTERFACE::PLANTRAJECTORY")
  (make-package "DEMO_INTERFACE::PLANTRAJECTORY"))
(unless (find-package "DEMO_INTERFACE::PLANTRAJECTORYREQUEST")
  (make-package "DEMO_INTERFACE::PLANTRAJECTORYREQUEST"))
(unless (find-package "DEMO_INTERFACE::PLANTRAJECTORYRESPONSE")
  (make-package "DEMO_INTERFACE::PLANTRAJECTORYRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(if (not (find-package "TRAJECTORY_MSGS"))
  (ros::roseus-add-msgs "trajectory_msgs"))


(defclass demo_interface::PlanTrajectoryRequest
  :super ros::object
  :slots (_target_pose _use_joints ))

(defmethod demo_interface::PlanTrajectoryRequest
  (:init
   (&key
    ((:target_pose __target_pose) (instance geometry_msgs::Pose :init))
    ((:use_joints __use_joints) nil)
    )
   (send-super :init)
   (setq _target_pose __target_pose)
   (setq _use_joints __use_joints)
   self)
  (:target_pose
   (&rest __target_pose)
   (if (keywordp (car __target_pose))
       (send* _target_pose __target_pose)
     (progn
       (if __target_pose (setq _target_pose (car __target_pose)))
       _target_pose)))
  (:use_joints
   (&optional (__use_joints :null))
   (if (not (eq __use_joints :null)) (setq _use_joints __use_joints)) _use_joints)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Pose _target_pose
    (send _target_pose :serialization-length)
    ;; bool _use_joints
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Pose _target_pose
       (send _target_pose :serialize s)
     ;; bool _use_joints
       (if _use_joints (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Pose _target_pose
     (send _target_pose :deserialize buf ptr-) (incf ptr- (send _target_pose :serialization-length))
   ;; bool _use_joints
     (setq _use_joints (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass demo_interface::PlanTrajectoryResponse
  :super ros::object
  :slots (_success _trajectory _planning_time _message ))

(defmethod demo_interface::PlanTrajectoryResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:trajectory __trajectory) (instance trajectory_msgs::JointTrajectory :init))
    ((:planning_time __planning_time) 0.0)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _trajectory __trajectory)
   (setq _planning_time (float __planning_time))
   (setq _message (string __message))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:trajectory
   (&rest __trajectory)
   (if (keywordp (car __trajectory))
       (send* _trajectory __trajectory)
     (progn
       (if __trajectory (setq _trajectory (car __trajectory)))
       _trajectory)))
  (:planning_time
   (&optional __planning_time)
   (if __planning_time (setq _planning_time __planning_time)) _planning_time)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; trajectory_msgs/JointTrajectory _trajectory
    (send _trajectory :serialization-length)
    ;; float32 _planning_time
    4
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; trajectory_msgs/JointTrajectory _trajectory
       (send _trajectory :serialize s)
     ;; float32 _planning_time
       (sys::poke _planning_time (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; trajectory_msgs/JointTrajectory _trajectory
     (send _trajectory :deserialize buf ptr-) (incf ptr- (send _trajectory :serialization-length))
   ;; float32 _planning_time
     (setq _planning_time (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass demo_interface::PlanTrajectory
  :super ros::object
  :slots ())

(setf (get demo_interface::PlanTrajectory :md5sum-) "07df15be4b07a5a918366cc9c76a3ccd")
(setf (get demo_interface::PlanTrajectory :datatype-) "demo_interface/PlanTrajectory")
(setf (get demo_interface::PlanTrajectory :request) demo_interface::PlanTrajectoryRequest)
(setf (get demo_interface::PlanTrajectory :response) demo_interface::PlanTrajectoryResponse)

(defmethod demo_interface::PlanTrajectoryRequest
  (:response () (instance demo_interface::PlanTrajectoryResponse :init)))

(setf (get demo_interface::PlanTrajectoryRequest :md5sum-) "07df15be4b07a5a918366cc9c76a3ccd")
(setf (get demo_interface::PlanTrajectoryRequest :datatype-) "demo_interface/PlanTrajectoryRequest")
(setf (get demo_interface::PlanTrajectoryRequest :definition-)
      "geometry_msgs/Pose target_pose
bool use_joints

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
---
bool success
trajectory_msgs/JointTrajectory trajectory
float32 planning_time
string message



================================================================================
MSG: trajectory_msgs/JointTrajectory
Header header
string[] joint_names
JointTrajectoryPoint[] points

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: trajectory_msgs/JointTrajectoryPoint
# Each trajectory point specifies either positions[, velocities[, accelerations]]
# or positions[, effort] for the trajectory to be executed.
# All specified values are in the same order as the joint names in JointTrajectory.msg

float64[] positions
float64[] velocities
float64[] accelerations
float64[] effort
duration time_from_start
")

(setf (get demo_interface::PlanTrajectoryResponse :md5sum-) "07df15be4b07a5a918366cc9c76a3ccd")
(setf (get demo_interface::PlanTrajectoryResponse :datatype-) "demo_interface/PlanTrajectoryResponse")
(setf (get demo_interface::PlanTrajectoryResponse :definition-)
      "geometry_msgs/Pose target_pose
bool use_joints

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
---
bool success
trajectory_msgs/JointTrajectory trajectory
float32 planning_time
string message



================================================================================
MSG: trajectory_msgs/JointTrajectory
Header header
string[] joint_names
JointTrajectoryPoint[] points

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: trajectory_msgs/JointTrajectoryPoint
# Each trajectory point specifies either positions[, velocities[, accelerations]]
# or positions[, effort] for the trajectory to be executed.
# All specified values are in the same order as the joint names in JointTrajectory.msg

float64[] positions
float64[] velocities
float64[] accelerations
float64[] effort
duration time_from_start
")



(provide :demo_interface/PlanTrajectory "07df15be4b07a5a918366cc9c76a3ccd")


