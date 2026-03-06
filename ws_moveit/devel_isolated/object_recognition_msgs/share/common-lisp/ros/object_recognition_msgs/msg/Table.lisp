; Auto-generated. Do not edit!


(cl:in-package object_recognition_msgs-msg)


;//! \htmlinclude Table.msg.html

(cl:defclass <Table> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (convex_hull
    :reader convex_hull
    :initarg :convex_hull
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point))))
)

(cl:defclass Table (<Table>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Table>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Table)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name object_recognition_msgs-msg:<Table> is deprecated: use object_recognition_msgs-msg:Table instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Table>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_recognition_msgs-msg:header-val is deprecated.  Use object_recognition_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <Table>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_recognition_msgs-msg:pose-val is deprecated.  Use object_recognition_msgs-msg:pose instead.")
  (pose m))

(cl:ensure-generic-function 'convex_hull-val :lambda-list '(m))
(cl:defmethod convex_hull-val ((m <Table>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_recognition_msgs-msg:convex_hull-val is deprecated.  Use object_recognition_msgs-msg:convex_hull instead.")
  (convex_hull m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Table>) ostream)
  "Serializes a message object of type '<Table>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'convex_hull))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'convex_hull))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Table>) istream)
  "Deserializes a message object of type '<Table>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'convex_hull) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'convex_hull)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Table>)))
  "Returns string type for a message object of type '<Table>"
  "object_recognition_msgs/Table")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Table)))
  "Returns string type for a message object of type 'Table"
  "object_recognition_msgs/Table")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Table>)))
  "Returns md5sum for a message object of type '<Table>"
  "39efebc7d51e44bd2d72f2df6c7823a2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Table)))
  "Returns md5sum for a message object of type 'Table"
  "39efebc7d51e44bd2d72f2df6c7823a2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Table>)))
  "Returns full string definition for message of type '<Table>"
  (cl:format cl:nil "# Informs that a planar table has been detected at a given location~%~%Header header~%~%# The pose gives you the transform that take you to the coordinate system~%# of the table, with the origin somewhere in the table plane and the ~%# z axis normal to the plane~%geometry_msgs/Pose pose~%~%# There is no guarantee that the table does NOT extend further than the~%# convex hull; this is just as far as we've observed it.~%# The origin of the table coordinate system is inside the convex hull~%~%# Set of points forming the convex hull of the table~%geometry_msgs/Point[] convex_hull~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Table)))
  "Returns full string definition for message of type 'Table"
  (cl:format cl:nil "# Informs that a planar table has been detected at a given location~%~%Header header~%~%# The pose gives you the transform that take you to the coordinate system~%# of the table, with the origin somewhere in the table plane and the ~%# z axis normal to the plane~%geometry_msgs/Pose pose~%~%# There is no guarantee that the table does NOT extend further than the~%# convex hull; this is just as far as we've observed it.~%# The origin of the table coordinate system is inside the convex hull~%~%# Set of points forming the convex hull of the table~%geometry_msgs/Point[] convex_hull~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Table>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'convex_hull) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Table>))
  "Converts a ROS message object to a list"
  (cl:list 'Table
    (cl:cons ':header (header msg))
    (cl:cons ':pose (pose msg))
    (cl:cons ':convex_hull (convex_hull msg))
))
