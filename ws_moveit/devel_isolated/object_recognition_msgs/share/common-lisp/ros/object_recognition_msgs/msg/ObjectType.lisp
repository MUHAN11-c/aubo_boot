; Auto-generated. Do not edit!


(cl:in-package object_recognition_msgs-msg)


;//! \htmlinclude ObjectType.msg.html

(cl:defclass <ObjectType> (roslisp-msg-protocol:ros-message)
  ((key
    :reader key
    :initarg :key
    :type cl:string
    :initform "")
   (db
    :reader db
    :initarg :db
    :type cl:string
    :initform ""))
)

(cl:defclass ObjectType (<ObjectType>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ObjectType>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ObjectType)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name object_recognition_msgs-msg:<ObjectType> is deprecated: use object_recognition_msgs-msg:ObjectType instead.")))

(cl:ensure-generic-function 'key-val :lambda-list '(m))
(cl:defmethod key-val ((m <ObjectType>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_recognition_msgs-msg:key-val is deprecated.  Use object_recognition_msgs-msg:key instead.")
  (key m))

(cl:ensure-generic-function 'db-val :lambda-list '(m))
(cl:defmethod db-val ((m <ObjectType>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_recognition_msgs-msg:db-val is deprecated.  Use object_recognition_msgs-msg:db instead.")
  (db m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ObjectType>) ostream)
  "Serializes a message object of type '<ObjectType>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'key))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'key))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'db))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'db))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ObjectType>) istream)
  "Deserializes a message object of type '<ObjectType>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'key) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'key) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'db) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'db) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ObjectType>)))
  "Returns string type for a message object of type '<ObjectType>"
  "object_recognition_msgs/ObjectType")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ObjectType)))
  "Returns string type for a message object of type 'ObjectType"
  "object_recognition_msgs/ObjectType")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ObjectType>)))
  "Returns md5sum for a message object of type '<ObjectType>"
  "ac757ec5be1998b0167e7efcda79e3cf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ObjectType)))
  "Returns md5sum for a message object of type 'ObjectType"
  "ac757ec5be1998b0167e7efcda79e3cf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ObjectType>)))
  "Returns full string definition for message of type '<ObjectType>"
  (cl:format cl:nil "################################################## OBJECT ID #########################################################~%~%# Contains information about the type of a found object. Those two sets of parameters together uniquely define an~%# object~%~%# The key of the found object: the unique identifier in the given db~%string key~%~%# The db parameters stored as a JSON/compressed YAML string. An object id does not make sense without the corresponding~%# database. E.g., in object_recognition, it can look like: \"{'type':'CouchDB', 'root':'http://localhost'}\"~%# There is no conventional format for those parameters and it's nice to keep that flexibility.~%# The object_recognition_core as a generic DB type that can read those fields~%# Current examples:~%# For CouchDB:~%#   type: 'CouchDB'~%#   root: 'http://localhost:5984'~%#   collection: 'object_recognition'~%# For SQL household database:~%#   type: 'SqlHousehold'~%#   host: 'wgs36'~%#   port: 5432~%#   user: 'willow'~%#   password: 'willow'~%#   name: 'household_objects'~%#   module: 'tabletop'~%string db~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ObjectType)))
  "Returns full string definition for message of type 'ObjectType"
  (cl:format cl:nil "################################################## OBJECT ID #########################################################~%~%# Contains information about the type of a found object. Those two sets of parameters together uniquely define an~%# object~%~%# The key of the found object: the unique identifier in the given db~%string key~%~%# The db parameters stored as a JSON/compressed YAML string. An object id does not make sense without the corresponding~%# database. E.g., in object_recognition, it can look like: \"{'type':'CouchDB', 'root':'http://localhost'}\"~%# There is no conventional format for those parameters and it's nice to keep that flexibility.~%# The object_recognition_core as a generic DB type that can read those fields~%# Current examples:~%# For CouchDB:~%#   type: 'CouchDB'~%#   root: 'http://localhost:5984'~%#   collection: 'object_recognition'~%# For SQL household database:~%#   type: 'SqlHousehold'~%#   host: 'wgs36'~%#   port: 5432~%#   user: 'willow'~%#   password: 'willow'~%#   name: 'household_objects'~%#   module: 'tabletop'~%string db~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ObjectType>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'key))
     4 (cl:length (cl:slot-value msg 'db))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ObjectType>))
  "Converts a ROS message object to a list"
  (cl:list 'ObjectType
    (cl:cons ':key (key msg))
    (cl:cons ':db (db msg))
))
