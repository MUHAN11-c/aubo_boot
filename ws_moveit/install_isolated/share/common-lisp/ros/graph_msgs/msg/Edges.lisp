; Auto-generated. Do not edit!


(cl:in-package graph_msgs-msg)


;//! \htmlinclude Edges.msg.html

(cl:defclass <Edges> (roslisp-msg-protocol:ros-message)
  ((node_ids
    :reader node_ids
    :initarg :node_ids
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (weights
    :reader weights
    :initarg :weights
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass Edges (<Edges>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Edges>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Edges)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name graph_msgs-msg:<Edges> is deprecated: use graph_msgs-msg:Edges instead.")))

(cl:ensure-generic-function 'node_ids-val :lambda-list '(m))
(cl:defmethod node_ids-val ((m <Edges>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader graph_msgs-msg:node_ids-val is deprecated.  Use graph_msgs-msg:node_ids instead.")
  (node_ids m))

(cl:ensure-generic-function 'weights-val :lambda-list '(m))
(cl:defmethod weights-val ((m <Edges>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader graph_msgs-msg:weights-val is deprecated.  Use graph_msgs-msg:weights instead.")
  (weights m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Edges>) ostream)
  "Serializes a message object of type '<Edges>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'node_ids))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) ele) ostream))
   (cl:slot-value msg 'node_ids))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'weights))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'weights))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Edges>) istream)
  "Deserializes a message object of type '<Edges>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'node_ids) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'node_ids)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:aref vals i)) (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'weights) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'weights)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Edges>)))
  "Returns string type for a message object of type '<Edges>"
  "graph_msgs/Edges")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Edges)))
  "Returns string type for a message object of type 'Edges"
  "graph_msgs/Edges")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Edges>)))
  "Returns md5sum for a message object of type '<Edges>"
  "1dcd54afd0b0c0fbebeb59dbdda4c026")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Edges)))
  "Returns md5sum for a message object of type 'Edges"
  "1dcd54afd0b0c0fbebeb59dbdda4c026")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Edges>)))
  "Returns full string definition for message of type '<Edges>"
  (cl:format cl:nil "#base-zero index of all the verticies this particular vertice connects to (edges)~%uint32[] node_ids~%~%# optional cost/weight of each edge. if vector is empty assume all weights are equal (1)~%float64[] weights~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Edges)))
  "Returns full string definition for message of type 'Edges"
  (cl:format cl:nil "#base-zero index of all the verticies this particular vertice connects to (edges)~%uint32[] node_ids~%~%# optional cost/weight of each edge. if vector is empty assume all weights are equal (1)~%float64[] weights~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Edges>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'node_ids) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'weights) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Edges>))
  "Converts a ROS message object to a list"
  (cl:list 'Edges
    (cl:cons ':node_ids (node_ids msg))
    (cl:cons ':weights (weights msg))
))
