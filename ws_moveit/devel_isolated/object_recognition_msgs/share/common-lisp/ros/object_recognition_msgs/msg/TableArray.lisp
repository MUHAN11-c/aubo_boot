; Auto-generated. Do not edit!


(cl:in-package object_recognition_msgs-msg)


;//! \htmlinclude TableArray.msg.html

(cl:defclass <TableArray> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (tables
    :reader tables
    :initarg :tables
    :type (cl:vector object_recognition_msgs-msg:Table)
   :initform (cl:make-array 0 :element-type 'object_recognition_msgs-msg:Table :initial-element (cl:make-instance 'object_recognition_msgs-msg:Table))))
)

(cl:defclass TableArray (<TableArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TableArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TableArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name object_recognition_msgs-msg:<TableArray> is deprecated: use object_recognition_msgs-msg:TableArray instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <TableArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_recognition_msgs-msg:header-val is deprecated.  Use object_recognition_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'tables-val :lambda-list '(m))
(cl:defmethod tables-val ((m <TableArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_recognition_msgs-msg:tables-val is deprecated.  Use object_recognition_msgs-msg:tables instead.")
  (tables m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TableArray>) ostream)
  "Serializes a message object of type '<TableArray>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'tables))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'tables))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TableArray>) istream)
  "Deserializes a message object of type '<TableArray>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'tables) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'tables)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'object_recognition_msgs-msg:Table))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TableArray>)))
  "Returns string type for a message object of type '<TableArray>"
  "object_recognition_msgs/TableArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TableArray)))
  "Returns string type for a message object of type 'TableArray"
  "object_recognition_msgs/TableArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TableArray>)))
  "Returns md5sum for a message object of type '<TableArray>"
  "d1c853e5acd0ed273eb6682dc01ab428")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TableArray)))
  "Returns md5sum for a message object of type 'TableArray"
  "d1c853e5acd0ed273eb6682dc01ab428")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TableArray>)))
  "Returns full string definition for message of type '<TableArray>"
  (cl:format cl:nil "Header header~%~%# Just an array of tables~%object_recognition_msgs/Table[] tables~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: object_recognition_msgs/Table~%# Informs that a planar table has been detected at a given location~%~%Header header~%~%# The pose gives you the transform that take you to the coordinate system~%# of the table, with the origin somewhere in the table plane and the ~%# z axis normal to the plane~%geometry_msgs/Pose pose~%~%# There is no guarantee that the table does NOT extend further than the~%# convex hull; this is just as far as we've observed it.~%# The origin of the table coordinate system is inside the convex hull~%~%# Set of points forming the convex hull of the table~%geometry_msgs/Point[] convex_hull~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TableArray)))
  "Returns full string definition for message of type 'TableArray"
  (cl:format cl:nil "Header header~%~%# Just an array of tables~%object_recognition_msgs/Table[] tables~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: object_recognition_msgs/Table~%# Informs that a planar table has been detected at a given location~%~%Header header~%~%# The pose gives you the transform that take you to the coordinate system~%# of the table, with the origin somewhere in the table plane and the ~%# z axis normal to the plane~%geometry_msgs/Pose pose~%~%# There is no guarantee that the table does NOT extend further than the~%# convex hull; this is just as far as we've observed it.~%# The origin of the table coordinate system is inside the convex hull~%~%# Set of points forming the convex hull of the table~%geometry_msgs/Point[] convex_hull~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TableArray>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'tables) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TableArray>))
  "Converts a ROS message object to a list"
  (cl:list 'TableArray
    (cl:cons ':header (header msg))
    (cl:cons ':tables (tables msg))
))
