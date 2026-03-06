;; Auto-generated. Do not edit!


(when (boundp 'object_recognition_msgs::RecognizedObject)
  (if (not (find-package "OBJECT_RECOGNITION_MSGS"))
    (make-package "OBJECT_RECOGNITION_MSGS"))
  (shadow 'RecognizedObject (find-package "OBJECT_RECOGNITION_MSGS")))
(unless (find-package "OBJECT_RECOGNITION_MSGS::RECOGNIZEDOBJECT")
  (make-package "OBJECT_RECOGNITION_MSGS::RECOGNIZEDOBJECT"))

(in-package "ROS")
;;//! \htmlinclude RecognizedObject.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))
(if (not (find-package "SHAPE_MSGS"))
  (ros::roseus-add-msgs "shape_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass object_recognition_msgs::RecognizedObject
  :super ros::object
  :slots (_header _type _confidence _point_clouds _bounding_mesh _bounding_contours _pose ))

(defmethod object_recognition_msgs::RecognizedObject
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:type __type) (instance object_recognition_msgs::ObjectType :init))
    ((:confidence __confidence) 0.0)
    ((:point_clouds __point_clouds) ())
    ((:bounding_mesh __bounding_mesh) (instance shape_msgs::Mesh :init))
    ((:bounding_contours __bounding_contours) ())
    ((:pose __pose) (instance geometry_msgs::PoseWithCovarianceStamped :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _type __type)
   (setq _confidence (float __confidence))
   (setq _point_clouds __point_clouds)
   (setq _bounding_mesh __bounding_mesh)
   (setq _bounding_contours __bounding_contours)
   (setq _pose __pose)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:type
   (&rest __type)
   (if (keywordp (car __type))
       (send* _type __type)
     (progn
       (if __type (setq _type (car __type)))
       _type)))
  (:confidence
   (&optional __confidence)
   (if __confidence (setq _confidence __confidence)) _confidence)
  (:point_clouds
   (&rest __point_clouds)
   (if (keywordp (car __point_clouds))
       (send* _point_clouds __point_clouds)
     (progn
       (if __point_clouds (setq _point_clouds (car __point_clouds)))
       _point_clouds)))
  (:bounding_mesh
   (&rest __bounding_mesh)
   (if (keywordp (car __bounding_mesh))
       (send* _bounding_mesh __bounding_mesh)
     (progn
       (if __bounding_mesh (setq _bounding_mesh (car __bounding_mesh)))
       _bounding_mesh)))
  (:bounding_contours
   (&rest __bounding_contours)
   (if (keywordp (car __bounding_contours))
       (send* _bounding_contours __bounding_contours)
     (progn
       (if __bounding_contours (setq _bounding_contours (car __bounding_contours)))
       _bounding_contours)))
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; object_recognition_msgs/ObjectType _type
    (send _type :serialization-length)
    ;; float32 _confidence
    4
    ;; sensor_msgs/PointCloud2[] _point_clouds
    (apply #'+ (send-all _point_clouds :serialization-length)) 4
    ;; shape_msgs/Mesh _bounding_mesh
    (send _bounding_mesh :serialization-length)
    ;; geometry_msgs/Point[] _bounding_contours
    (apply #'+ (send-all _bounding_contours :serialization-length)) 4
    ;; geometry_msgs/PoseWithCovarianceStamped _pose
    (send _pose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; object_recognition_msgs/ObjectType _type
       (send _type :serialize s)
     ;; float32 _confidence
       (sys::poke _confidence (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; sensor_msgs/PointCloud2[] _point_clouds
     (write-long (length _point_clouds) s)
     (dolist (elem _point_clouds)
       (send elem :serialize s)
       )
     ;; shape_msgs/Mesh _bounding_mesh
       (send _bounding_mesh :serialize s)
     ;; geometry_msgs/Point[] _bounding_contours
     (write-long (length _bounding_contours) s)
     (dolist (elem _bounding_contours)
       (send elem :serialize s)
       )
     ;; geometry_msgs/PoseWithCovarianceStamped _pose
       (send _pose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; object_recognition_msgs/ObjectType _type
     (send _type :deserialize buf ptr-) (incf ptr- (send _type :serialization-length))
   ;; float32 _confidence
     (setq _confidence (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; sensor_msgs/PointCloud2[] _point_clouds
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _point_clouds (let (r) (dotimes (i n) (push (instance sensor_msgs::PointCloud2 :init) r)) r))
     (dolist (elem- _point_clouds)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; shape_msgs/Mesh _bounding_mesh
     (send _bounding_mesh :deserialize buf ptr-) (incf ptr- (send _bounding_mesh :serialization-length))
   ;; geometry_msgs/Point[] _bounding_contours
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _bounding_contours (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _bounding_contours)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/PoseWithCovarianceStamped _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;;
   self)
  )

(setf (get object_recognition_msgs::RecognizedObject :md5sum-) "f92c4cb29ba11f26c5f7219de97e900d")
(setf (get object_recognition_msgs::RecognizedObject :datatype-) "object_recognition_msgs/RecognizedObject")
(setf (get object_recognition_msgs::RecognizedObject :definition-)
      "##################################################### HEADER ###########################################################

# The header frame corresponds to the pose frame, NOT the point_cloud frame.
Header header

################################################## OBJECT INFO #########################################################

# Contains information about the type and the position of a found object
# Some of those fields might not be filled because the used techniques do not fill them or because the user does not
# request them

# The type of the found object
object_recognition_msgs/ObjectType type

#confidence: how sure you are it is that object and not another one.
# It is between 0 and 1 and the closer to one it is the better
float32 confidence

################################################ OBJECT CLUSTERS #######################################################

# Sometimes you can extract the 3d points that belong to the object, in the frames of the original sensors
# (it is an array as you might have several sensors)
sensor_msgs/PointCloud2[] point_clouds

# Sometimes, you can only provide a bounding box/shape, even in 3d
# This is in the pose frame
shape_msgs/Mesh bounding_mesh

# Sometimes, you only have 2d input so you can't really give a pose, you just get a contour, or a box
# The last point will be linked to the first one automatically
geometry_msgs/Point[] bounding_contours

#################################################### POSE INFO #########################################################

# This is the result that everybody expects : the pose in some frame given with the input. The units are radian/meters
# as usual
geometry_msgs/PoseWithCovarianceStamped pose

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
MSG: geometry_msgs/PoseWithCovarianceStamped
# This expresses an estimated pose with a reference coordinate frame and timestamp

Header header
PoseWithCovariance pose

================================================================================
MSG: geometry_msgs/PoseWithCovariance
# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :object_recognition_msgs/RecognizedObject "f92c4cb29ba11f26c5f7219de97e900d")


