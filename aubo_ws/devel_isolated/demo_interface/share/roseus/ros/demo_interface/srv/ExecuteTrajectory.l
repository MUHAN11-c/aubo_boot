;; Auto-generated. Do not edit!


(when (boundp 'demo_interface::ExecuteTrajectory)
  (if (not (find-package "DEMO_INTERFACE"))
    (make-package "DEMO_INTERFACE"))
  (shadow 'ExecuteTrajectory (find-package "DEMO_INTERFACE")))
(unless (find-package "DEMO_INTERFACE::EXECUTETRAJECTORY")
  (make-package "DEMO_INTERFACE::EXECUTETRAJECTORY"))
(unless (find-package "DEMO_INTERFACE::EXECUTETRAJECTORYREQUEST")
  (make-package "DEMO_INTERFACE::EXECUTETRAJECTORYREQUEST"))
(unless (find-package "DEMO_INTERFACE::EXECUTETRAJECTORYRESPONSE")
  (make-package "DEMO_INTERFACE::EXECUTETRAJECTORYRESPONSE"))

(in-package "ROS")

(if (not (find-package "TRAJECTORY_MSGS"))
  (ros::roseus-add-msgs "trajectory_msgs"))




(defclass demo_interface::ExecuteTrajectoryRequest
  :super ros::object
  :slots (_trajectory ))

(defmethod demo_interface::ExecuteTrajectoryRequest
  (:init
   (&key
    ((:trajectory __trajectory) (instance trajectory_msgs::JointTrajectory :init))
    )
   (send-super :init)
   (setq _trajectory __trajectory)
   self)
  (:trajectory
   (&rest __trajectory)
   (if (keywordp (car __trajectory))
       (send* _trajectory __trajectory)
     (progn
       (if __trajectory (setq _trajectory (car __trajectory)))
       _trajectory)))
  (:serialization-length
   ()
   (+
    ;; trajectory_msgs/JointTrajectory _trajectory
    (send _trajectory :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; trajectory_msgs/JointTrajectory _trajectory
       (send _trajectory :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; trajectory_msgs/JointTrajectory _trajectory
     (send _trajectory :deserialize buf ptr-) (incf ptr- (send _trajectory :serialization-length))
   ;;
   self)
  )

(defclass demo_interface::ExecuteTrajectoryResponse
  :super ros::object
  :slots (_success _error_code _message ))

(defmethod demo_interface::ExecuteTrajectoryResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:error_code __error_code) 0)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _error_code (round __error_code))
   (setq _message (string __message))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:error_code
   (&optional __error_code)
   (if __error_code (setq _error_code __error_code)) _error_code)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; int32 _error_code
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
     ;; int32 _error_code
       (write-long _error_code s)
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int32 _error_code
     (setq _error_code (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass demo_interface::ExecuteTrajectory
  :super ros::object
  :slots ())

(setf (get demo_interface::ExecuteTrajectory :md5sum-) "141b3c90cc65183a9295539645f2c064")
(setf (get demo_interface::ExecuteTrajectory :datatype-) "demo_interface/ExecuteTrajectory")
(setf (get demo_interface::ExecuteTrajectory :request) demo_interface::ExecuteTrajectoryRequest)
(setf (get demo_interface::ExecuteTrajectory :response) demo_interface::ExecuteTrajectoryResponse)

(defmethod demo_interface::ExecuteTrajectoryRequest
  (:response () (instance demo_interface::ExecuteTrajectoryResponse :init)))

(setf (get demo_interface::ExecuteTrajectoryRequest :md5sum-) "141b3c90cc65183a9295539645f2c064")
(setf (get demo_interface::ExecuteTrajectoryRequest :datatype-) "demo_interface/ExecuteTrajectoryRequest")
(setf (get demo_interface::ExecuteTrajectoryRequest :definition-)
      "trajectory_msgs/JointTrajectory trajectory

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
---
bool success
int32 error_code
string message


")

(setf (get demo_interface::ExecuteTrajectoryResponse :md5sum-) "141b3c90cc65183a9295539645f2c064")
(setf (get demo_interface::ExecuteTrajectoryResponse :datatype-) "demo_interface/ExecuteTrajectoryResponse")
(setf (get demo_interface::ExecuteTrajectoryResponse :definition-)
      "trajectory_msgs/JointTrajectory trajectory

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
---
bool success
int32 error_code
string message


")



(provide :demo_interface/ExecuteTrajectory "141b3c90cc65183a9295539645f2c064")


