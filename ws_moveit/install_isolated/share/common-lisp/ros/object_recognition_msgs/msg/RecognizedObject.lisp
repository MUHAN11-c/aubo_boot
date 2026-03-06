; Auto-generated. Do not edit!


(cl:in-package object_recognition_msgs-msg)


;//! \htmlinclude RecognizedObject.msg.html

(cl:defclass <RecognizedObject> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (type
    :reader type
    :initarg :type
    :type object_recognition_msgs-msg:ObjectType
    :initform (cl:make-instance 'object_recognition_msgs-msg:ObjectType))
   (confidence
    :reader confidence
    :initarg :confidence
    :type cl:float
    :initform 0.0)
   (point_clouds
    :reader point_clouds
    :initarg :point_clouds
    :type (cl:vector sensor_msgs-msg:PointCloud2)
   :initform (cl:make-array 0 :element-type 'sensor_msgs-msg:PointCloud2 :initial-element (cl:make-instance 'sensor_msgs-msg:PointCloud2)))
   (bounding_mesh
    :reader bounding_mesh
    :initarg :bounding_mesh
    :type shape_msgs-msg:Mesh
    :initform (cl:make-instance 'shape_msgs-msg:Mesh))
   (bounding_contours
    :reader bounding_contours
    :initarg :bounding_contours
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point)))
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:PoseWithCovarianceStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseWithCovarianceStamped)))
)

(cl:defclass RecognizedObject (<RecognizedObject>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RecognizedObject>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RecognizedObject)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name object_recognition_msgs-msg:<RecognizedObject> is deprecated: use object_recognition_msgs-msg:RecognizedObject instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <RecognizedObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_recognition_msgs-msg:header-val is deprecated.  Use object_recognition_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <RecognizedObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_recognition_msgs-msg:type-val is deprecated.  Use object_recognition_msgs-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'confidence-val :lambda-list '(m))
(cl:defmethod confidence-val ((m <RecognizedObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_recognition_msgs-msg:confidence-val is deprecated.  Use object_recognition_msgs-msg:confidence instead.")
  (confidence m))

(cl:ensure-generic-function 'point_clouds-val :lambda-list '(m))
(cl:defmethod point_clouds-val ((m <RecognizedObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_recognition_msgs-msg:point_clouds-val is deprecated.  Use object_recognition_msgs-msg:point_clouds instead.")
  (point_clouds m))

(cl:ensure-generic-function 'bounding_mesh-val :lambda-list '(m))
(cl:defmethod bounding_mesh-val ((m <RecognizedObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_recognition_msgs-msg:bounding_mesh-val is deprecated.  Use object_recognition_msgs-msg:bounding_mesh instead.")
  (bounding_mesh m))

(cl:ensure-generic-function 'bounding_contours-val :lambda-list '(m))
(cl:defmethod bounding_contours-val ((m <RecognizedObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_recognition_msgs-msg:bounding_contours-val is deprecated.  Use object_recognition_msgs-msg:bounding_contours instead.")
  (bounding_contours m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <RecognizedObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_recognition_msgs-msg:pose-val is deprecated.  Use object_recognition_msgs-msg:pose instead.")
  (pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RecognizedObject>) ostream)
  "Serializes a message object of type '<RecognizedObject>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'type) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'confidence))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'point_clouds))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'point_clouds))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'bounding_mesh) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'bounding_contours))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'bounding_contours))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RecognizedObject>) istream)
  "Deserializes a message object of type '<RecognizedObject>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'type) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'confidence) (roslisp-utils:decode-single-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'point_clouds) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'point_clouds)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'sensor_msgs-msg:PointCloud2))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'bounding_mesh) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'bounding_contours) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'bounding_contours)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RecognizedObject>)))
  "Returns string type for a message object of type '<RecognizedObject>"
  "object_recognition_msgs/RecognizedObject")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RecognizedObject)))
  "Returns string type for a message object of type 'RecognizedObject"
  "object_recognition_msgs/RecognizedObject")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RecognizedObject>)))
  "Returns md5sum for a message object of type '<RecognizedObject>"
  "f92c4cb29ba11f26c5f7219de97e900d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RecognizedObject)))
  "Returns md5sum for a message object of type 'RecognizedObject"
  "f92c4cb29ba11f26c5f7219de97e900d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RecognizedObject>)))
  "Returns full string definition for message of type '<RecognizedObject>"
  (cl:format cl:nil "##################################################### HEADER ###########################################################~%~%# The header frame corresponds to the pose frame, NOT the point_cloud frame.~%Header header~%~%################################################## OBJECT INFO #########################################################~%~%# Contains information about the type and the position of a found object~%# Some of those fields might not be filled because the used techniques do not fill them or because the user does not~%# request them~%~%# The type of the found object~%object_recognition_msgs/ObjectType type~%~%#confidence: how sure you are it is that object and not another one.~%# It is between 0 and 1 and the closer to one it is the better~%float32 confidence~%~%################################################ OBJECT CLUSTERS #######################################################~%~%# Sometimes you can extract the 3d points that belong to the object, in the frames of the original sensors~%# (it is an array as you might have several sensors)~%sensor_msgs/PointCloud2[] point_clouds~%~%# Sometimes, you can only provide a bounding box/shape, even in 3d~%# This is in the pose frame~%shape_msgs/Mesh bounding_mesh~%~%# Sometimes, you only have 2d input so you can't really give a pose, you just get a contour, or a box~%# The last point will be linked to the first one automatically~%geometry_msgs/Point[] bounding_contours~%~%#################################################### POSE INFO #########################################################~%~%# This is the result that everybody expects : the pose in some frame given with the input. The units are radian/meters~%# as usual~%geometry_msgs/PoseWithCovarianceStamped pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: object_recognition_msgs/ObjectType~%################################################## OBJECT ID #########################################################~%~%# Contains information about the type of a found object. Those two sets of parameters together uniquely define an~%# object~%~%# The key of the found object: the unique identifier in the given db~%string key~%~%# The db parameters stored as a JSON/compressed YAML string. An object id does not make sense without the corresponding~%# database. E.g., in object_recognition, it can look like: \"{'type':'CouchDB', 'root':'http://localhost'}\"~%# There is no conventional format for those parameters and it's nice to keep that flexibility.~%# The object_recognition_core as a generic DB type that can read those fields~%# Current examples:~%# For CouchDB:~%#   type: 'CouchDB'~%#   root: 'http://localhost:5984'~%#   collection: 'object_recognition'~%# For SQL household database:~%#   type: 'SqlHousehold'~%#   host: 'wgs36'~%#   port: 5432~%#   user: 'willow'~%#   password: 'willow'~%#   name: 'household_objects'~%#   module: 'tabletop'~%string db~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%================================================================================~%MSG: shape_msgs/Mesh~%# Definition of a mesh~%~%# list of triangles; the index values refer to positions in vertices[]~%MeshTriangle[] triangles~%~%# the actual vertices that make up the mesh~%geometry_msgs/Point[] vertices~%~%================================================================================~%MSG: shape_msgs/MeshTriangle~%# Definition of a triangle's vertices~%uint32[3] vertex_indices~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovarianceStamped~%# This expresses an estimated pose with a reference coordinate frame and timestamp~%~%Header header~%PoseWithCovariance pose~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RecognizedObject)))
  "Returns full string definition for message of type 'RecognizedObject"
  (cl:format cl:nil "##################################################### HEADER ###########################################################~%~%# The header frame corresponds to the pose frame, NOT the point_cloud frame.~%Header header~%~%################################################## OBJECT INFO #########################################################~%~%# Contains information about the type and the position of a found object~%# Some of those fields might not be filled because the used techniques do not fill them or because the user does not~%# request them~%~%# The type of the found object~%object_recognition_msgs/ObjectType type~%~%#confidence: how sure you are it is that object and not another one.~%# It is between 0 and 1 and the closer to one it is the better~%float32 confidence~%~%################################################ OBJECT CLUSTERS #######################################################~%~%# Sometimes you can extract the 3d points that belong to the object, in the frames of the original sensors~%# (it is an array as you might have several sensors)~%sensor_msgs/PointCloud2[] point_clouds~%~%# Sometimes, you can only provide a bounding box/shape, even in 3d~%# This is in the pose frame~%shape_msgs/Mesh bounding_mesh~%~%# Sometimes, you only have 2d input so you can't really give a pose, you just get a contour, or a box~%# The last point will be linked to the first one automatically~%geometry_msgs/Point[] bounding_contours~%~%#################################################### POSE INFO #########################################################~%~%# This is the result that everybody expects : the pose in some frame given with the input. The units are radian/meters~%# as usual~%geometry_msgs/PoseWithCovarianceStamped pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: object_recognition_msgs/ObjectType~%################################################## OBJECT ID #########################################################~%~%# Contains information about the type of a found object. Those two sets of parameters together uniquely define an~%# object~%~%# The key of the found object: the unique identifier in the given db~%string key~%~%# The db parameters stored as a JSON/compressed YAML string. An object id does not make sense without the corresponding~%# database. E.g., in object_recognition, it can look like: \"{'type':'CouchDB', 'root':'http://localhost'}\"~%# There is no conventional format for those parameters and it's nice to keep that flexibility.~%# The object_recognition_core as a generic DB type that can read those fields~%# Current examples:~%# For CouchDB:~%#   type: 'CouchDB'~%#   root: 'http://localhost:5984'~%#   collection: 'object_recognition'~%# For SQL household database:~%#   type: 'SqlHousehold'~%#   host: 'wgs36'~%#   port: 5432~%#   user: 'willow'~%#   password: 'willow'~%#   name: 'household_objects'~%#   module: 'tabletop'~%string db~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%================================================================================~%MSG: shape_msgs/Mesh~%# Definition of a mesh~%~%# list of triangles; the index values refer to positions in vertices[]~%MeshTriangle[] triangles~%~%# the actual vertices that make up the mesh~%geometry_msgs/Point[] vertices~%~%================================================================================~%MSG: shape_msgs/MeshTriangle~%# Definition of a triangle's vertices~%uint32[3] vertex_indices~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovarianceStamped~%# This expresses an estimated pose with a reference coordinate frame and timestamp~%~%Header header~%PoseWithCovariance pose~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RecognizedObject>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'type))
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'point_clouds) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'bounding_mesh))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'bounding_contours) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RecognizedObject>))
  "Converts a ROS message object to a list"
  (cl:list 'RecognizedObject
    (cl:cons ':header (header msg))
    (cl:cons ':type (type msg))
    (cl:cons ':confidence (confidence msg))
    (cl:cons ':point_clouds (point_clouds msg))
    (cl:cons ':bounding_mesh (bounding_mesh msg))
    (cl:cons ':bounding_contours (bounding_contours msg))
    (cl:cons ':pose (pose msg))
))
