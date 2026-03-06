; Auto-generated. Do not edit!


(cl:in-package graph_msgs-msg)


;//! \htmlinclude GeometryGraph.msg.html

(cl:defclass <GeometryGraph> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (nodes
    :reader nodes
    :initarg :nodes
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point)))
   (edges
    :reader edges
    :initarg :edges
    :type (cl:vector graph_msgs-msg:Edges)
   :initform (cl:make-array 0 :element-type 'graph_msgs-msg:Edges :initial-element (cl:make-instance 'graph_msgs-msg:Edges))))
)

(cl:defclass GeometryGraph (<GeometryGraph>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GeometryGraph>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GeometryGraph)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name graph_msgs-msg:<GeometryGraph> is deprecated: use graph_msgs-msg:GeometryGraph instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <GeometryGraph>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader graph_msgs-msg:header-val is deprecated.  Use graph_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'nodes-val :lambda-list '(m))
(cl:defmethod nodes-val ((m <GeometryGraph>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader graph_msgs-msg:nodes-val is deprecated.  Use graph_msgs-msg:nodes instead.")
  (nodes m))

(cl:ensure-generic-function 'edges-val :lambda-list '(m))
(cl:defmethod edges-val ((m <GeometryGraph>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader graph_msgs-msg:edges-val is deprecated.  Use graph_msgs-msg:edges instead.")
  (edges m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GeometryGraph>) ostream)
  "Serializes a message object of type '<GeometryGraph>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'nodes))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'nodes))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'edges))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'edges))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GeometryGraph>) istream)
  "Deserializes a message object of type '<GeometryGraph>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'nodes) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'nodes)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'edges) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'edges)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'graph_msgs-msg:Edges))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GeometryGraph>)))
  "Returns string type for a message object of type '<GeometryGraph>"
  "graph_msgs/GeometryGraph")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GeometryGraph)))
  "Returns string type for a message object of type 'GeometryGraph"
  "graph_msgs/GeometryGraph")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GeometryGraph>)))
  "Returns md5sum for a message object of type '<GeometryGraph>"
  "78739617daca94d38915923795eada2d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GeometryGraph)))
  "Returns md5sum for a message object of type 'GeometryGraph"
  "78739617daca94d38915923795eada2d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GeometryGraph>)))
  "Returns full string definition for message of type '<GeometryGraph>"
  (cl:format cl:nil "# A reference coordinate frame and timestamp~%Header header~%~%# 3D spacial graph~%geometry_msgs/Point[] nodes~%~%# This vector should be the same length as nodes, above, and represents all the connecting nodes~%Edges[] edges~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: graph_msgs/Edges~%#base-zero index of all the verticies this particular vertice connects to (edges)~%uint32[] node_ids~%~%# optional cost/weight of each edge. if vector is empty assume all weights are equal (1)~%float64[] weights~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GeometryGraph)))
  "Returns full string definition for message of type 'GeometryGraph"
  (cl:format cl:nil "# A reference coordinate frame and timestamp~%Header header~%~%# 3D spacial graph~%geometry_msgs/Point[] nodes~%~%# This vector should be the same length as nodes, above, and represents all the connecting nodes~%Edges[] edges~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: graph_msgs/Edges~%#base-zero index of all the verticies this particular vertice connects to (edges)~%uint32[] node_ids~%~%# optional cost/weight of each edge. if vector is empty assume all weights are equal (1)~%float64[] weights~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GeometryGraph>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'nodes) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'edges) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GeometryGraph>))
  "Converts a ROS message object to a list"
  (cl:list 'GeometryGraph
    (cl:cons ':header (header msg))
    (cl:cons ':nodes (nodes msg))
    (cl:cons ':edges (edges msg))
))
