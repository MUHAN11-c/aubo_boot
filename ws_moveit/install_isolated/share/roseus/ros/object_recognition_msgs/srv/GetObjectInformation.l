;; Auto-generated. Do not edit!


(when (boundp 'object_recognition_msgs::GetObjectInformation)
  (if (not (find-package "OBJECT_RECOGNITION_MSGS"))
    (make-package "OBJECT_RECOGNITION_MSGS"))
  (shadow 'GetObjectInformation (find-package "OBJECT_RECOGNITION_MSGS")))
(unless (find-package "OBJECT_RECOGNITION_MSGS::GETOBJECTINFORMATION")
  (make-package "OBJECT_RECOGNITION_MSGS::GETOBJECTINFORMATION"))
(unless (find-package "OBJECT_RECOGNITION_MSGS::GETOBJECTINFORMATIONREQUEST")
  (make-package "OBJECT_RECOGNITION_MSGS::GETOBJECTINFORMATIONREQUEST"))
(unless (find-package "OBJECT_RECOGNITION_MSGS::GETOBJECTINFORMATIONRESPONSE")
  (make-package "OBJECT_RECOGNITION_MSGS::GETOBJECTINFORMATIONRESPONSE"))

(in-package "ROS")





(defclass object_recognition_msgs::GetObjectInformationRequest
  :super ros::object
  :slots (_type ))

(defmethod object_recognition_msgs::GetObjectInformationRequest
  (:init
   (&key
    ((:type __type) (instance object_recognition_msgs::ObjectType :init))
    )
   (send-super :init)
   (setq _type __type)
   self)
  (:type
   (&rest __type)
   (if (keywordp (car __type))
       (send* _type __type)
     (progn
       (if __type (setq _type (car __type)))
       _type)))
  (:serialization-length
   ()
   (+
    ;; object_recognition_msgs/ObjectType _type
    (send _type :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; object_recognition_msgs/ObjectType _type
       (send _type :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; object_recognition_msgs/ObjectType _type
     (send _type :deserialize buf ptr-) (incf ptr- (send _type :serialization-length))
   ;;
   self)
  )

(defclass object_recognition_msgs::GetObjectInformationResponse
  :super ros::object
  :slots (_information ))

(defmethod object_recognition_msgs::GetObjectInformationResponse
  (:init
   (&key
    ((:information __information) (instance object_recognition_msgs::ObjectInformation :init))
    )
   (send-super :init)
   (setq _information __information)
   self)
  (:information
   (&rest __information)
   (if (keywordp (car __information))
       (send* _information __information)
     (progn
       (if __information (setq _information (car __information)))
       _information)))
  (:serialization-length
   ()
   (+
    ;; object_recognition_msgs/ObjectInformation _information
    (send _information :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; object_recognition_msgs/ObjectInformation _information
       (send _information :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; object_recognition_msgs/ObjectInformation _information
     (send _information :deserialize buf ptr-) (incf ptr- (send _information :serialization-length))
   ;;
   self)
  )

(defclass object_recognition_msgs::GetObjectInformation
  :super ros::object
  :slots ())

(setf (get object_recognition_msgs::GetObjectInformation :md5sum-) "dd7d344324fd86c32836f4fe1bc7b322")
(setf (get object_recognition_msgs::GetObjectInformation :datatype-) "object_recognition_msgs/GetObjectInformation")
(setf (get object_recognition_msgs::GetObjectInformation :request) object_recognition_msgs::GetObjectInformationRequest)
(setf (get object_recognition_msgs::GetObjectInformation :response) object_recognition_msgs::GetObjectInformationResponse)

(defmethod object_recognition_msgs::GetObjectInformationRequest
  (:response () (instance object_recognition_msgs::GetObjectInformationResponse :init)))

(setf (get object_recognition_msgs::GetObjectInformationRequest :md5sum-) "dd7d344324fd86c32836f4fe1bc7b322")
(setf (get object_recognition_msgs::GetObjectInformationRequest :datatype-) "object_recognition_msgs/GetObjectInformationRequest")
(setf (get object_recognition_msgs::GetObjectInformationRequest :definition-)
      "# Retrieve extra data from the DB for a given object

# The type of the object to retrieve info from
object_recognition_msgs/ObjectType type


================================================================================
MSG: object_recognition_msgs/ObjectType
################################################## OBJECT ID #########################################################

# Contains information about the type of a found object. Those two sets of parameters together uniquely define an
# object

# The key of the found object: the unique identifier in the given db
string key

# The db parameters stored as a JSON/compressed YAML string. An object id does not make sense without the corresponding
# database. E.g., in object_recognition, it can look like: \"{'type':'CouchDB', 'root':'http://localhost'}\"
# There is no conventional format for those parameters and it's nice to keep that flexibility.
# The object_recognition_core as a generic DB type that can read those fields
# Current examples:
# For CouchDB:
#   type: 'CouchDB'
#   root: 'http://localhost:5984'
#   collection: 'object_recognition'
# For SQL household database:
#   type: 'SqlHousehold'
#   host: 'wgs36'
#   port: 5432
#   user: 'willow'
#   password: 'willow'
#   name: 'household_objects'
#   module: 'tabletop'
string db
---

# Extra object info 
object_recognition_msgs/ObjectInformation information


================================================================================
MSG: object_recognition_msgs/ObjectInformation
############################################## VISUALIZATION INFO ######################################################
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

(setf (get object_recognition_msgs::GetObjectInformationResponse :md5sum-) "dd7d344324fd86c32836f4fe1bc7b322")
(setf (get object_recognition_msgs::GetObjectInformationResponse :datatype-) "object_recognition_msgs/GetObjectInformationResponse")
(setf (get object_recognition_msgs::GetObjectInformationResponse :definition-)
      "# Retrieve extra data from the DB for a given object

# The type of the object to retrieve info from
object_recognition_msgs/ObjectType type


================================================================================
MSG: object_recognition_msgs/ObjectType
################################################## OBJECT ID #########################################################

# Contains information about the type of a found object. Those two sets of parameters together uniquely define an
# object

# The key of the found object: the unique identifier in the given db
string key

# The db parameters stored as a JSON/compressed YAML string. An object id does not make sense without the corresponding
# database. E.g., in object_recognition, it can look like: \"{'type':'CouchDB', 'root':'http://localhost'}\"
# There is no conventional format for those parameters and it's nice to keep that flexibility.
# The object_recognition_core as a generic DB type that can read those fields
# Current examples:
# For CouchDB:
#   type: 'CouchDB'
#   root: 'http://localhost:5984'
#   collection: 'object_recognition'
# For SQL household database:
#   type: 'SqlHousehold'
#   host: 'wgs36'
#   port: 5432
#   user: 'willow'
#   password: 'willow'
#   name: 'household_objects'
#   module: 'tabletop'
string db
---

# Extra object info 
object_recognition_msgs/ObjectInformation information


================================================================================
MSG: object_recognition_msgs/ObjectInformation
############################################## VISUALIZATION INFO ######################################################
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



(provide :object_recognition_msgs/GetObjectInformation "dd7d344324fd86c32836f4fe1bc7b322")


