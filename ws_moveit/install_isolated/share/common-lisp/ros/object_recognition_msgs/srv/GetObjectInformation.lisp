; Auto-generated. Do not edit!


(cl:in-package object_recognition_msgs-srv)


;//! \htmlinclude GetObjectInformation-request.msg.html

(cl:defclass <GetObjectInformation-request> (roslisp-msg-protocol:ros-message)
  ((type
    :reader type
    :initarg :type
    :type object_recognition_msgs-msg:ObjectType
    :initform (cl:make-instance 'object_recognition_msgs-msg:ObjectType)))
)

(cl:defclass GetObjectInformation-request (<GetObjectInformation-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetObjectInformation-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetObjectInformation-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name object_recognition_msgs-srv:<GetObjectInformation-request> is deprecated: use object_recognition_msgs-srv:GetObjectInformation-request instead.")))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <GetObjectInformation-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_recognition_msgs-srv:type-val is deprecated.  Use object_recognition_msgs-srv:type instead.")
  (type m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetObjectInformation-request>) ostream)
  "Serializes a message object of type '<GetObjectInformation-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'type) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetObjectInformation-request>) istream)
  "Deserializes a message object of type '<GetObjectInformation-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'type) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetObjectInformation-request>)))
  "Returns string type for a service object of type '<GetObjectInformation-request>"
  "object_recognition_msgs/GetObjectInformationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetObjectInformation-request)))
  "Returns string type for a service object of type 'GetObjectInformation-request"
  "object_recognition_msgs/GetObjectInformationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetObjectInformation-request>)))
  "Returns md5sum for a message object of type '<GetObjectInformation-request>"
  "dd7d344324fd86c32836f4fe1bc7b322")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetObjectInformation-request)))
  "Returns md5sum for a message object of type 'GetObjectInformation-request"
  "dd7d344324fd86c32836f4fe1bc7b322")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetObjectInformation-request>)))
  "Returns full string definition for message of type '<GetObjectInformation-request>"
  (cl:format cl:nil "# Retrieve extra data from the DB for a given object~%~%# The type of the object to retrieve info from~%object_recognition_msgs/ObjectType type~%~%~%================================================================================~%MSG: object_recognition_msgs/ObjectType~%################################################## OBJECT ID #########################################################~%~%# Contains information about the type of a found object. Those two sets of parameters together uniquely define an~%# object~%~%# The key of the found object: the unique identifier in the given db~%string key~%~%# The db parameters stored as a JSON/compressed YAML string. An object id does not make sense without the corresponding~%# database. E.g., in object_recognition, it can look like: \"{'type':'CouchDB', 'root':'http://localhost'}\"~%# There is no conventional format for those parameters and it's nice to keep that flexibility.~%# The object_recognition_core as a generic DB type that can read those fields~%# Current examples:~%# For CouchDB:~%#   type: 'CouchDB'~%#   root: 'http://localhost:5984'~%#   collection: 'object_recognition'~%# For SQL household database:~%#   type: 'SqlHousehold'~%#   host: 'wgs36'~%#   port: 5432~%#   user: 'willow'~%#   password: 'willow'~%#   name: 'household_objects'~%#   module: 'tabletop'~%string db~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetObjectInformation-request)))
  "Returns full string definition for message of type 'GetObjectInformation-request"
  (cl:format cl:nil "# Retrieve extra data from the DB for a given object~%~%# The type of the object to retrieve info from~%object_recognition_msgs/ObjectType type~%~%~%================================================================================~%MSG: object_recognition_msgs/ObjectType~%################################################## OBJECT ID #########################################################~%~%# Contains information about the type of a found object. Those two sets of parameters together uniquely define an~%# object~%~%# The key of the found object: the unique identifier in the given db~%string key~%~%# The db parameters stored as a JSON/compressed YAML string. An object id does not make sense without the corresponding~%# database. E.g., in object_recognition, it can look like: \"{'type':'CouchDB', 'root':'http://localhost'}\"~%# There is no conventional format for those parameters and it's nice to keep that flexibility.~%# The object_recognition_core as a generic DB type that can read those fields~%# Current examples:~%# For CouchDB:~%#   type: 'CouchDB'~%#   root: 'http://localhost:5984'~%#   collection: 'object_recognition'~%# For SQL household database:~%#   type: 'SqlHousehold'~%#   host: 'wgs36'~%#   port: 5432~%#   user: 'willow'~%#   password: 'willow'~%#   name: 'household_objects'~%#   module: 'tabletop'~%string db~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetObjectInformation-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'type))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetObjectInformation-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetObjectInformation-request
    (cl:cons ':type (type msg))
))
;//! \htmlinclude GetObjectInformation-response.msg.html

(cl:defclass <GetObjectInformation-response> (roslisp-msg-protocol:ros-message)
  ((information
    :reader information
    :initarg :information
    :type object_recognition_msgs-msg:ObjectInformation
    :initform (cl:make-instance 'object_recognition_msgs-msg:ObjectInformation)))
)

(cl:defclass GetObjectInformation-response (<GetObjectInformation-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetObjectInformation-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetObjectInformation-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name object_recognition_msgs-srv:<GetObjectInformation-response> is deprecated: use object_recognition_msgs-srv:GetObjectInformation-response instead.")))

(cl:ensure-generic-function 'information-val :lambda-list '(m))
(cl:defmethod information-val ((m <GetObjectInformation-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_recognition_msgs-srv:information-val is deprecated.  Use object_recognition_msgs-srv:information instead.")
  (information m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetObjectInformation-response>) ostream)
  "Serializes a message object of type '<GetObjectInformation-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'information) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetObjectInformation-response>) istream)
  "Deserializes a message object of type '<GetObjectInformation-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'information) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetObjectInformation-response>)))
  "Returns string type for a service object of type '<GetObjectInformation-response>"
  "object_recognition_msgs/GetObjectInformationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetObjectInformation-response)))
  "Returns string type for a service object of type 'GetObjectInformation-response"
  "object_recognition_msgs/GetObjectInformationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetObjectInformation-response>)))
  "Returns md5sum for a message object of type '<GetObjectInformation-response>"
  "dd7d344324fd86c32836f4fe1bc7b322")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetObjectInformation-response)))
  "Returns md5sum for a message object of type 'GetObjectInformation-response"
  "dd7d344324fd86c32836f4fe1bc7b322")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetObjectInformation-response>)))
  "Returns full string definition for message of type '<GetObjectInformation-response>"
  (cl:format cl:nil "~%# Extra object info ~%object_recognition_msgs/ObjectInformation information~%~%~%================================================================================~%MSG: object_recognition_msgs/ObjectInformation~%############################################## VISUALIZATION INFO ######################################################~%################### THIS INFO SHOULD BE OBTAINED INDEPENDENTLY FROM THE CORE, LIKE IN AN RVIZ PLUGIN ###################~%~%# The human readable name of the object~%string name~%~%# The full mesh of the object: this can be useful for display purposes, augmented reality ... but it can be big~%# Make sure the type is MESH~%shape_msgs/Mesh ground_truth_mesh~%~%# Sometimes, you only have a cloud in the DB~%# Make sure the type is POINTS~%sensor_msgs/PointCloud2 ground_truth_point_cloud~%~%================================================================================~%MSG: shape_msgs/Mesh~%# Definition of a mesh~%~%# list of triangles; the index values refer to positions in vertices[]~%MeshTriangle[] triangles~%~%# the actual vertices that make up the mesh~%geometry_msgs/Point[] vertices~%~%================================================================================~%MSG: shape_msgs/MeshTriangle~%# Definition of a triangle's vertices~%uint32[3] vertex_indices~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetObjectInformation-response)))
  "Returns full string definition for message of type 'GetObjectInformation-response"
  (cl:format cl:nil "~%# Extra object info ~%object_recognition_msgs/ObjectInformation information~%~%~%================================================================================~%MSG: object_recognition_msgs/ObjectInformation~%############################################## VISUALIZATION INFO ######################################################~%################### THIS INFO SHOULD BE OBTAINED INDEPENDENTLY FROM THE CORE, LIKE IN AN RVIZ PLUGIN ###################~%~%# The human readable name of the object~%string name~%~%# The full mesh of the object: this can be useful for display purposes, augmented reality ... but it can be big~%# Make sure the type is MESH~%shape_msgs/Mesh ground_truth_mesh~%~%# Sometimes, you only have a cloud in the DB~%# Make sure the type is POINTS~%sensor_msgs/PointCloud2 ground_truth_point_cloud~%~%================================================================================~%MSG: shape_msgs/Mesh~%# Definition of a mesh~%~%# list of triangles; the index values refer to positions in vertices[]~%MeshTriangle[] triangles~%~%# the actual vertices that make up the mesh~%geometry_msgs/Point[] vertices~%~%================================================================================~%MSG: shape_msgs/MeshTriangle~%# Definition of a triangle's vertices~%uint32[3] vertex_indices~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetObjectInformation-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'information))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetObjectInformation-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetObjectInformation-response
    (cl:cons ':information (information msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetObjectInformation)))
  'GetObjectInformation-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetObjectInformation)))
  'GetObjectInformation-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetObjectInformation)))
  "Returns string type for a service object of type '<GetObjectInformation>"
  "object_recognition_msgs/GetObjectInformation")