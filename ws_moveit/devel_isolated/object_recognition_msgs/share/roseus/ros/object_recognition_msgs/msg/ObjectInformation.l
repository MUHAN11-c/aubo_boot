;; Auto-generated. Do not edit!


(when (boundp 'object_recognition_msgs::ObjectInformation)
  (if (not (find-package "OBJECT_RECOGNITION_MSGS"))
    (make-package "OBJECT_RECOGNITION_MSGS"))
  (shadow 'ObjectInformation (find-package "OBJECT_RECOGNITION_MSGS")))
(unless (find-package "OBJECT_RECOGNITION_MSGS::OBJECTINFORMATION")
  (make-package "OBJECT_RECOGNITION_MSGS::OBJECTINFORMATION"))

(in-package "ROS")
;;//! \htmlinclude ObjectInformation.msg.html
(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))
(if (not (find-package "SHAPE_MSGS"))
  (ros::roseus-add-msgs "shape_msgs"))


(defclass object_recognition_msgs::ObjectInformation
  :super ros::object
  :slots (_name _ground_truth_mesh _ground_truth_point_cloud ))

(defmethod object_recognition_msgs::ObjectInformation
  (:init
   (&key
    ((:name __name) "")
    ((:ground_truth_mesh __ground_truth_mesh) (instance shape_msgs::Mesh :init))
    ((:ground_truth_point_cloud __ground_truth_point_cloud) (instance sensor_msgs::PointCloud2 :init))
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _ground_truth_mesh __ground_truth_mesh)
   (setq _ground_truth_point_cloud __ground_truth_point_cloud)
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:ground_truth_mesh
   (&rest __ground_truth_mesh)
   (if (keywordp (car __ground_truth_mesh))
       (send* _ground_truth_mesh __ground_truth_mesh)
     (progn
       (if __ground_truth_mesh (setq _ground_truth_mesh (car __ground_truth_mesh)))
       _ground_truth_mesh)))
  (:ground_truth_point_cloud
   (&rest __ground_truth_point_cloud)
   (if (keywordp (car __ground_truth_point_cloud))
       (send* _ground_truth_point_cloud __ground_truth_point_cloud)
     (progn
       (if __ground_truth_point_cloud (setq _ground_truth_point_cloud (car __ground_truth_point_cloud)))
       _ground_truth_point_cloud)))
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; shape_msgs/Mesh _ground_truth_mesh
    (send _ground_truth_mesh :serialization-length)
    ;; sensor_msgs/PointCloud2 _ground_truth_point_cloud
    (send _ground_truth_point_cloud :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; shape_msgs/Mesh _ground_truth_mesh
       (send _ground_truth_mesh :serialize s)
     ;; sensor_msgs/PointCloud2 _ground_truth_point_cloud
       (send _ground_truth_point_cloud :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; shape_msgs/Mesh _ground_truth_mesh
     (send _ground_truth_mesh :deserialize buf ptr-) (incf ptr- (send _ground_truth_mesh :serialization-length))
   ;; sensor_msgs/PointCloud2 _ground_truth_point_cloud
     (send _ground_truth_point_cloud :deserialize buf ptr-) (incf ptr- (send _ground_truth_point_cloud :serialization-length))
   ;;
   self)
  )

(setf (get object_recognition_msgs::ObjectInformation :md5sum-) "921ec39f51c7b927902059cf3300ecde")
(setf (get object_recognition_msgs::ObjectInformation :datatype-) "object_recognition_msgs/ObjectInformation")
(setf (get object_recognition_msgs::ObjectInformation :definition-)
      "############################################## VISUALIZATION INFO ######################################################
################### THIS INFO SHOULD BE OBTAINED INDEPENDENTLY FROM THE CORE, LIKE IN AN RVIZ PLUGIN ###################

# The human readable name of the object
string name

# The full mesh of the object: this can be useful for display purposes, augmented reality ... but it can be big
# Make sure the type is MESH
shape_msgs/Mesh ground_truth_mesh

# Sometimes, you only have a cloud in the DB
# Make sure the type is POINTS
sensor_msgs/PointCloud2 ground_truth_point_cloud

================================================================================
MSG: shape_msgs/Mesh
# Definition of a mesh

# list of triangles; the index values refer to positions in vertices[]
MeshTriangle[] triangles

# the actual vertices that make up the mesh
geometry_msgs/Point[] vertices

================================================================================
MSG: shape_msgs/MeshTriangle
# Definition of a triangle's vertices
uint32[3] vertex_indices

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: sensor_msgs/PointCloud2
# This message holds a collection of N-dimensional points, which may
# contain additional information such as normals, intensity, etc. The
# point data is stored as a binary blob, its layout described by the
# contents of the \"fields\" array.

# The point cloud data may be organized 2d (image-like) or 1d
# (unordered). Point clouds organized as 2d images may be produced by
# camera depth sensors such as stereo or time-of-flight.

# Time of sensor data acquisition, and the coordinate frame ID (for 3d
# points).
Header header

# 2D structure of the point cloud. If the cloud is unordered, height is
# 1 and width is the length of the point cloud.
uint32 height
uint32 width

# Describes the channels and their layout in the binary data blob.
PointField[] fields

bool    is_bigendian # Is this data bigendian?
uint32  point_step   # Length of a point in bytes
uint32  row_step     # Length of a row in bytes
uint8[] data         # Actual point data, size is (row_step*height)

bool is_dense        # True if there are no invalid points

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
MSG: sensor_msgs/PointField
# This message holds the description of one point entry in the
# PointCloud2 message format.
uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6
uint8 FLOAT32 = 7
uint8 FLOAT64 = 8

string name      # Name of field
uint32 offset    # Offset from start of point struct
uint8  datatype  # Datatype enumeration, see above
uint32 count     # How many elements in the field

")



(provide :object_recognition_msgs/ObjectInformation "921ec39f51c7b927902059cf3300ecde")


