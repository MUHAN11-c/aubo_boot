; Auto-generated. Do not edit!


(cl:in-package object_recognition_msgs-msg)


;//! \htmlinclude ObjectInformation.msg.html

(cl:defclass <ObjectInformation> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (ground_truth_mesh
    :reader ground_truth_mesh
    :initarg :ground_truth_mesh
    :type shape_msgs-msg:Mesh
    :initform (cl:make-instance 'shape_msgs-msg:Mesh))
   (ground_truth_point_cloud
    :reader ground_truth_point_cloud
    :initarg :ground_truth_point_cloud
    :type sensor_msgs-msg:PointCloud2
    :initform (cl:make-instance 'sensor_msgs-msg:PointCloud2)))
)

(cl:defclass ObjectInformation (<ObjectInformation>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ObjectInformation>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ObjectInformation)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name object_recognition_msgs-msg:<ObjectInformation> is deprecated: use object_recognition_msgs-msg:ObjectInformation instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <ObjectInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_recognition_msgs-msg:name-val is deprecated.  Use object_recognition_msgs-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'ground_truth_mesh-val :lambda-list '(m))
(cl:defmethod ground_truth_mesh-val ((m <ObjectInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_recognition_msgs-msg:ground_truth_mesh-val is deprecated.  Use object_recognition_msgs-msg:ground_truth_mesh instead.")
  (ground_truth_mesh m))

(cl:ensure-generic-function 'ground_truth_point_cloud-val :lambda-list '(m))
(cl:defmethod ground_truth_point_cloud-val ((m <ObjectInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_recognition_msgs-msg:ground_truth_point_cloud-val is deprecated.  Use object_recognition_msgs-msg:ground_truth_point_cloud instead.")
  (ground_truth_point_cloud m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ObjectInformation>) ostream)
  "Serializes a message object of type '<ObjectInformation>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'ground_truth_mesh) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'ground_truth_point_cloud) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ObjectInformation>) istream)
  "Deserializes a message object of type '<ObjectInformation>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'ground_truth_mesh) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'ground_truth_point_cloud) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ObjectInformation>)))
  "Returns string type for a message object of type '<ObjectInformation>"
  "object_recognition_msgs/ObjectInformation")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ObjectInformation)))
  "Returns string type for a message object of type 'ObjectInformation"
  "object_recognition_msgs/ObjectInformation")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ObjectInformation>)))
  "Returns md5sum for a message object of type '<ObjectInformation>"
  "921ec39f51c7b927902059cf3300ecde")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ObjectInformation)))
  "Returns md5sum for a message object of type 'ObjectInformation"
  "921ec39f51c7b927902059cf3300ecde")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ObjectInformation>)))
  "Returns full string definition for message of type '<ObjectInformation>"
  (cl:format cl:nil "############################################## VISUALIZATION INFO ######################################################~%################### THIS INFO SHOULD BE OBTAINED INDEPENDENTLY FROM THE CORE, LIKE IN AN RVIZ PLUGIN ###################~%~%# The human readable name of the object~%string name~%~%# The full mesh of the object: this can be useful for display purposes, augmented reality ... but it can be big~%# Make sure the type is MESH~%shape_msgs/Mesh ground_truth_mesh~%~%# Sometimes, you only have a cloud in the DB~%# Make sure the type is POINTS~%sensor_msgs/PointCloud2 ground_truth_point_cloud~%~%================================================================================~%MSG: shape_msgs/Mesh~%# Definition of a mesh~%~%# list of triangles; the index values refer to positions in vertices[]~%MeshTriangle[] triangles~%~%# the actual vertices that make up the mesh~%geometry_msgs/Point[] vertices~%~%================================================================================~%MSG: shape_msgs/MeshTriangle~%# Definition of a triangle's vertices~%uint32[3] vertex_indices~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ObjectInformation)))
  "Returns full string definition for message of type 'ObjectInformation"
  (cl:format cl:nil "############################################## VISUALIZATION INFO ######################################################~%################### THIS INFO SHOULD BE OBTAINED INDEPENDENTLY FROM THE CORE, LIKE IN AN RVIZ PLUGIN ###################~%~%# The human readable name of the object~%string name~%~%# The full mesh of the object: this can be useful for display purposes, augmented reality ... but it can be big~%# Make sure the type is MESH~%shape_msgs/Mesh ground_truth_mesh~%~%# Sometimes, you only have a cloud in the DB~%# Make sure the type is POINTS~%sensor_msgs/PointCloud2 ground_truth_point_cloud~%~%================================================================================~%MSG: shape_msgs/Mesh~%# Definition of a mesh~%~%# list of triangles; the index values refer to positions in vertices[]~%MeshTriangle[] triangles~%~%# the actual vertices that make up the mesh~%geometry_msgs/Point[] vertices~%~%================================================================================~%MSG: shape_msgs/MeshTriangle~%# Definition of a triangle's vertices~%uint32[3] vertex_indices~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ObjectInformation>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'ground_truth_mesh))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'ground_truth_point_cloud))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ObjectInformation>))
  "Converts a ROS message object to a list"
  (cl:list 'ObjectInformation
    (cl:cons ':name (name msg))
    (cl:cons ':ground_truth_mesh (ground_truth_mesh msg))
    (cl:cons ':ground_truth_point_cloud (ground_truth_point_cloud msg))
))
