;; Auto-generated. Do not edit!


(when (boundp 'object_recognition_msgs::TableArray)
  (if (not (find-package "OBJECT_RECOGNITION_MSGS"))
    (make-package "OBJECT_RECOGNITION_MSGS"))
  (shadow 'TableArray (find-package "OBJECT_RECOGNITION_MSGS")))
(unless (find-package "OBJECT_RECOGNITION_MSGS::TABLEARRAY")
  (make-package "OBJECT_RECOGNITION_MSGS::TABLEARRAY"))

(in-package "ROS")
;;//! \htmlinclude TableArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass object_recognition_msgs::TableArray
  :super ros::object
  :slots (_header _tables ))

(defmethod object_recognition_msgs::TableArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:tables __tables) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _tables __tables)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:tables
   (&rest __tables)
   (if (keywordp (car __tables))
       (send* _tables __tables)
     (progn
       (if __tables (setq _tables (car __tables)))
       _tables)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; object_recognition_msgs/Table[] _tables
    (apply #'+ (send-all _tables :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; object_recognition_msgs/Table[] _tables
     (write-long (length _tables) s)
     (dolist (elem _tables)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; object_recognition_msgs/Table[] _tables
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _tables (let (r) (dotimes (i n) (push (instance object_recognition_msgs::Table :init) r)) r))
     (dolist (elem- _tables)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get object_recognition_msgs::TableArray :md5sum-) "d1c853e5acd0ed273eb6682dc01ab428")
(setf (get object_recognition_msgs::TableArray :datatype-) "object_recognition_msgs/TableArray")
(setf (get object_recognition_msgs::TableArray :definition-)
      "Header header

# Just an array of tables
object_recognition_msgs/Table[] tables

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
MSG: object_recognition_msgs/Table
# Informs that a planar table has been detected at a given location

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



(provide :object_recognition_msgs/TableArray "d1c853e5acd0ed273eb6682dc01ab428")


