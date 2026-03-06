;; Auto-generated. Do not edit!


(when (boundp 'object_recognition_msgs::Table)
  (if (not (find-package "OBJECT_RECOGNITION_MSGS"))
    (make-package "OBJECT_RECOGNITION_MSGS"))
  (shadow 'Table (find-package "OBJECT_RECOGNITION_MSGS")))
(unless (find-package "OBJECT_RECOGNITION_MSGS::TABLE")
  (make-package "OBJECT_RECOGNITION_MSGS::TABLE"))

(in-package "ROS")
;;//! \htmlinclude Table.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass object_recognition_msgs::Table
  :super ros::object
  :slots (_header _pose _convex_hull ))

(defmethod object_recognition_msgs::Table
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:pose __pose) (instance geometry_msgs::Pose :init))
    ((:convex_hull __convex_hull) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _pose __pose)
   (setq _convex_hull __convex_hull)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:convex_hull
   (&rest __convex_hull)
   (if (keywordp (car __convex_hull))
       (send* _convex_hull __convex_hull)
     (progn
       (if __convex_hull (setq _convex_hull (car __convex_hull)))
       _convex_hull)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Pose _pose
    (send _pose :serialization-length)
    ;; geometry_msgs/Point[] _convex_hull
    (apply #'+ (send-all _convex_hull :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Pose _pose
       (send _pose :serialize s)
     ;; geometry_msgs/Point[] _convex_hull
     (write-long (length _convex_hull) s)
     (dolist (elem _convex_hull)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Pose _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;; geometry_msgs/Point[] _convex_hull
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _convex_hull (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _convex_hull)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get object_recognition_msgs::Table :md5sum-) "39efebc7d51e44bd2d72f2df6c7823a2")
(setf (get object_recognition_msgs::Table :datatype-) "object_recognition_msgs/Table")
(setf (get object_recognition_msgs::Table :definition-)
      "# Informs that a planar table has been detected at a given location

Header header

# The pose gives you the transform that take you to the coordinate system
# of the table, with the origin somewhere in the table plane and the 
# z axis normal to the plane
geometry_msgs/Pose pose

# There is no guarantee that the table does NOT extend further than the
# convex hull; this is just as far as we've observed it.
# The origin of the table coordinate system is inside the convex hull

# Set of points forming the convex hull of the table
geometry_msgs/Point[] convex_hull

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



(provide :object_recognition_msgs/Table "39efebc7d51e44bd2d72f2df6c7823a2")


