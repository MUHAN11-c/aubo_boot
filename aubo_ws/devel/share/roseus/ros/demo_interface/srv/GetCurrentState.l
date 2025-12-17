;; Auto-generated. Do not edit!


(when (boundp 'demo_interface::GetCurrentState)
  (if (not (find-package "DEMO_INTERFACE"))
    (make-package "DEMO_INTERFACE"))
  (shadow 'GetCurrentState (find-package "DEMO_INTERFACE")))
(unless (find-package "DEMO_INTERFACE::GETCURRENTSTATE")
  (make-package "DEMO_INTERFACE::GETCURRENTSTATE"))
(unless (find-package "DEMO_INTERFACE::GETCURRENTSTATEREQUEST")
  (make-package "DEMO_INTERFACE::GETCURRENTSTATEREQUEST"))
(unless (find-package "DEMO_INTERFACE::GETCURRENTSTATERESPONSE")
  (make-package "DEMO_INTERFACE::GETCURRENTSTATERESPONSE"))

(in-package "ROS")



(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass demo_interface::GetCurrentStateRequest
  :super ros::object
  :slots ())

(defmethod demo_interface::GetCurrentStateRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass demo_interface::GetCurrentStateResponse
  :super ros::object
  :slots (_success _joint_position_rad _cartesian_position _velocity _message ))

(defmethod demo_interface::GetCurrentStateResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:joint_position_rad __joint_position_rad) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:cartesian_position __cartesian_position) (instance geometry_msgs::Pose :init))
    ((:velocity __velocity) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:message __message) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _joint_position_rad __joint_position_rad)
   (setq _cartesian_position __cartesian_position)
   (setq _velocity __velocity)
   (setq _message (string __message))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:joint_position_rad
   (&optional __joint_position_rad)
   (if __joint_position_rad (setq _joint_position_rad __joint_position_rad)) _joint_position_rad)
  (:cartesian_position
   (&rest __cartesian_position)
   (if (keywordp (car __cartesian_position))
       (send* _cartesian_position __cartesian_position)
     (progn
       (if __cartesian_position (setq _cartesian_position (car __cartesian_position)))
       _cartesian_position)))
  (:velocity
   (&optional __velocity)
   (if __velocity (setq _velocity __velocity)) _velocity)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; float64[] _joint_position_rad
    (* 8    (length _joint_position_rad)) 4
    ;; geometry_msgs/Pose _cartesian_position
    (send _cartesian_position :serialization-length)
    ;; float64[] _velocity
    (* 8    (length _velocity)) 4
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; float64[] _joint_position_rad
     (write-long (length _joint_position_rad) s)
     (dotimes (i (length _joint_position_rad))
       (sys::poke (elt _joint_position_rad i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; geometry_msgs/Pose _cartesian_position
       (send _cartesian_position :serialize s)
     ;; float64[] _velocity
     (write-long (length _velocity) s)
     (dotimes (i (length _velocity))
       (sys::poke (elt _velocity i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64[] _joint_position_rad
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_position_rad (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _joint_position_rad i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; geometry_msgs/Pose _cartesian_position
     (send _cartesian_position :deserialize buf ptr-) (incf ptr- (send _cartesian_position :serialization-length))
   ;; float64[] _velocity
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _velocity (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _velocity i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass demo_interface::GetCurrentState
  :super ros::object
  :slots ())

(setf (get demo_interface::GetCurrentState :md5sum-) "54871f675804e280e9a4e64601214b7d")
(setf (get demo_interface::GetCurrentState :datatype-) "demo_interface/GetCurrentState")
(setf (get demo_interface::GetCurrentState :request) demo_interface::GetCurrentStateRequest)
(setf (get demo_interface::GetCurrentState :response) demo_interface::GetCurrentStateResponse)

(defmethod demo_interface::GetCurrentStateRequest
  (:response () (instance demo_interface::GetCurrentStateResponse :init)))

(setf (get demo_interface::GetCurrentStateRequest :md5sum-) "54871f675804e280e9a4e64601214b7d")
(setf (get demo_interface::GetCurrentStateRequest :datatype-) "demo_interface/GetCurrentStateRequest")
(setf (get demo_interface::GetCurrentStateRequest :definition-)
      "---
bool success
float64[] joint_position_rad
geometry_msgs/Pose cartesian_position
float64[] velocity
string message



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

(setf (get demo_interface::GetCurrentStateResponse :md5sum-) "54871f675804e280e9a4e64601214b7d")
(setf (get demo_interface::GetCurrentStateResponse :datatype-) "demo_interface/GetCurrentStateResponse")
(setf (get demo_interface::GetCurrentStateResponse :definition-)
      "---
bool success
float64[] joint_position_rad
geometry_msgs/Pose cartesian_position
float64[] velocity
string message



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



(provide :demo_interface/GetCurrentState "54871f675804e280e9a4e64601214b7d")


