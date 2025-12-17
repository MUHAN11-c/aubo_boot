;; Auto-generated. Do not edit!


(when (boundp 'demo_interface::MoveToPose)
  (if (not (find-package "DEMO_INTERFACE"))
    (make-package "DEMO_INTERFACE"))
  (shadow 'MoveToPose (find-package "DEMO_INTERFACE")))
(unless (find-package "DEMO_INTERFACE::MOVETOPOSE")
  (make-package "DEMO_INTERFACE::MOVETOPOSE"))
(unless (find-package "DEMO_INTERFACE::MOVETOPOSEREQUEST")
  (make-package "DEMO_INTERFACE::MOVETOPOSEREQUEST"))
(unless (find-package "DEMO_INTERFACE::MOVETOPOSERESPONSE")
  (make-package "DEMO_INTERFACE::MOVETOPOSERESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass demo_interface::MoveToPoseRequest
  :super ros::object
  :slots (_target_pose _use_joints _velocity_factor _acceleration_factor ))

(defmethod demo_interface::MoveToPoseRequest
  (:init
   (&key
    ((:target_pose __target_pose) (instance geometry_msgs::Pose :init))
    ((:use_joints __use_joints) nil)
    ((:velocity_factor __velocity_factor) 0.0)
    ((:acceleration_factor __acceleration_factor) 0.0)
    )
   (send-super :init)
   (setq _target_pose __target_pose)
   (setq _use_joints __use_joints)
   (setq _velocity_factor (float __velocity_factor))
   (setq _acceleration_factor (float __acceleration_factor))
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
  (:velocity_factor
   (&optional __velocity_factor)
   (if __velocity_factor (setq _velocity_factor __velocity_factor)) _velocity_factor)
  (:acceleration_factor
   (&optional __acceleration_factor)
   (if __acceleration_factor (setq _acceleration_factor __acceleration_factor)) _acceleration_factor)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Pose _target_pose
    (send _target_pose :serialization-length)
    ;; bool _use_joints
    1
    ;; float32 _velocity_factor
    4
    ;; float32 _acceleration_factor
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Pose _target_pose
       (send _target_pose :serialize s)
     ;; bool _use_joints
       (if _use_joints (write-byte -1 s) (write-byte 0 s))
     ;; float32 _velocity_factor
       (sys::poke _velocity_factor (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _acceleration_factor
       (sys::poke _acceleration_factor (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Pose _target_pose
     (send _target_pose :deserialize buf ptr-) (incf ptr- (send _target_pose :serialization-length))
   ;; bool _use_joints
     (setq _use_joints (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _velocity_factor
     (setq _velocity_factor (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _acceleration_factor
     (setq _acceleration_factor (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass demo_interface::MoveToPoseResponse
  :super ros::object
  :slots (_success _error_code _message ))

(defmethod demo_interface::MoveToPoseResponse
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

(defclass demo_interface::MoveToPose
  :super ros::object
  :slots ())

(setf (get demo_interface::MoveToPose :md5sum-) "13d651572c46e71643945d975979b6fc")
(setf (get demo_interface::MoveToPose :datatype-) "demo_interface/MoveToPose")
(setf (get demo_interface::MoveToPose :request) demo_interface::MoveToPoseRequest)
(setf (get demo_interface::MoveToPose :response) demo_interface::MoveToPoseResponse)

(defmethod demo_interface::MoveToPoseRequest
  (:response () (instance demo_interface::MoveToPoseResponse :init)))

(setf (get demo_interface::MoveToPoseRequest :md5sum-) "13d651572c46e71643945d975979b6fc")
(setf (get demo_interface::MoveToPoseRequest :datatype-) "demo_interface/MoveToPoseRequest")
(setf (get demo_interface::MoveToPoseRequest :definition-)
      "geometry_msgs/Pose target_pose
bool use_joints
float32 velocity_factor
float32 acceleration_factor

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
int32 error_code
string message


")

(setf (get demo_interface::MoveToPoseResponse :md5sum-) "13d651572c46e71643945d975979b6fc")
(setf (get demo_interface::MoveToPoseResponse :datatype-) "demo_interface/MoveToPoseResponse")
(setf (get demo_interface::MoveToPoseResponse :definition-)
      "geometry_msgs/Pose target_pose
bool use_joints
float32 velocity_factor
float32 acceleration_factor

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
int32 error_code
string message


")



(provide :demo_interface/MoveToPose "13d651572c46e71643945d975979b6fc")


