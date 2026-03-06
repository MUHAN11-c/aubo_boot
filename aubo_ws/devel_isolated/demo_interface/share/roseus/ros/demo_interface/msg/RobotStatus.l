;; Auto-generated. Do not edit!


(when (boundp 'demo_interface::RobotStatus)
  (if (not (find-package "DEMO_INTERFACE"))
    (make-package "DEMO_INTERFACE"))
  (shadow 'RobotStatus (find-package "DEMO_INTERFACE")))
(unless (find-package "DEMO_INTERFACE::ROBOTSTATUS")
  (make-package "DEMO_INTERFACE::ROBOTSTATUS"))

(in-package "ROS")
;;//! \htmlinclude RobotStatus.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass demo_interface::RobotStatus
  :super ros::object
  :slots (_header _is_online _enable _in_motion _planning_status _joint_position_rad _joint_position_deg _cartesian_position ))

(defmethod demo_interface::RobotStatus
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:is_online __is_online) nil)
    ((:enable __enable) nil)
    ((:in_motion __in_motion) nil)
    ((:planning_status __planning_status) "")
    ((:joint_position_rad __joint_position_rad) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:joint_position_deg __joint_position_deg) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:cartesian_position __cartesian_position) (instance geometry_msgs::Pose :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _is_online __is_online)
   (setq _enable __enable)
   (setq _in_motion __in_motion)
   (setq _planning_status (string __planning_status))
   (setq _joint_position_rad __joint_position_rad)
   (setq _joint_position_deg __joint_position_deg)
   (setq _cartesian_position __cartesian_position)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:is_online
   (&optional (__is_online :null))
   (if (not (eq __is_online :null)) (setq _is_online __is_online)) _is_online)
  (:enable
   (&optional (__enable :null))
   (if (not (eq __enable :null)) (setq _enable __enable)) _enable)
  (:in_motion
   (&optional (__in_motion :null))
   (if (not (eq __in_motion :null)) (setq _in_motion __in_motion)) _in_motion)
  (:planning_status
   (&optional __planning_status)
   (if __planning_status (setq _planning_status __planning_status)) _planning_status)
  (:joint_position_rad
   (&optional __joint_position_rad)
   (if __joint_position_rad (setq _joint_position_rad __joint_position_rad)) _joint_position_rad)
  (:joint_position_deg
   (&optional __joint_position_deg)
   (if __joint_position_deg (setq _joint_position_deg __joint_position_deg)) _joint_position_deg)
  (:cartesian_position
   (&rest __cartesian_position)
   (if (keywordp (car __cartesian_position))
       (send* _cartesian_position __cartesian_position)
     (progn
       (if __cartesian_position (setq _cartesian_position (car __cartesian_position)))
       _cartesian_position)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _is_online
    1
    ;; bool _enable
    1
    ;; bool _in_motion
    1
    ;; string _planning_status
    4 (length _planning_status)
    ;; float64[6] _joint_position_rad
    (* 8    6)
    ;; float64[6] _joint_position_deg
    (* 8    6)
    ;; geometry_msgs/Pose _cartesian_position
    (send _cartesian_position :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _is_online
       (if _is_online (write-byte -1 s) (write-byte 0 s))
     ;; bool _enable
       (if _enable (write-byte -1 s) (write-byte 0 s))
     ;; bool _in_motion
       (if _in_motion (write-byte -1 s) (write-byte 0 s))
     ;; string _planning_status
       (write-long (length _planning_status) s) (princ _planning_status s)
     ;; float64[6] _joint_position_rad
     (dotimes (i 6)
       (sys::poke (elt _joint_position_rad i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _joint_position_deg
     (dotimes (i 6)
       (sys::poke (elt _joint_position_deg i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; geometry_msgs/Pose _cartesian_position
       (send _cartesian_position :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _is_online
     (setq _is_online (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _enable
     (setq _enable (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _in_motion
     (setq _in_motion (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _planning_status
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _planning_status (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64[6] _joint_position_rad
   (dotimes (i (length _joint_position_rad))
     (setf (elt _joint_position_rad i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _joint_position_deg
   (dotimes (i (length _joint_position_deg))
     (setf (elt _joint_position_deg i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; geometry_msgs/Pose _cartesian_position
     (send _cartesian_position :deserialize buf ptr-) (incf ptr- (send _cartesian_position :serialization-length))
   ;;
   self)
  )

(setf (get demo_interface::RobotStatus :md5sum-) "5cf9bff7fe374641a90823ee8d3a68eb")
(setf (get demo_interface::RobotStatus :datatype-) "demo_interface/RobotStatus")
(setf (get demo_interface::RobotStatus :definition-)
      "std_msgs/Header header
bool is_online
bool enable
bool in_motion
string planning_status
float64[6] joint_position_rad
float64[6] joint_position_deg
geometry_msgs/Pose cartesian_position


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

")



(provide :demo_interface/RobotStatus "5cf9bff7fe374641a90823ee8d3a68eb")


