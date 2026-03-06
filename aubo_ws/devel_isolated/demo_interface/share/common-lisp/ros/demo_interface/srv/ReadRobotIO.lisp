; Auto-generated. Do not edit!


(cl:in-package demo_interface-srv)


;//! \htmlinclude ReadRobotIO-request.msg.html

(cl:defclass <ReadRobotIO-request> (roslisp-msg-protocol:ros-message)
  ((io_type
    :reader io_type
    :initarg :io_type
    :type cl:string
    :initform "")
   (io_index
    :reader io_index
    :initarg :io_index
    :type cl:integer
    :initform 0))
)

(cl:defclass ReadRobotIO-request (<ReadRobotIO-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReadRobotIO-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReadRobotIO-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name demo_interface-srv:<ReadRobotIO-request> is deprecated: use demo_interface-srv:ReadRobotIO-request instead.")))

(cl:ensure-generic-function 'io_type-val :lambda-list '(m))
(cl:defmethod io_type-val ((m <ReadRobotIO-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader demo_interface-srv:io_type-val is deprecated.  Use demo_interface-srv:io_type instead.")
  (io_type m))

(cl:ensure-generic-function 'io_index-val :lambda-list '(m))
(cl:defmethod io_index-val ((m <ReadRobotIO-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader demo_interface-srv:io_index-val is deprecated.  Use demo_interface-srv:io_index instead.")
  (io_index m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReadRobotIO-request>) ostream)
  "Serializes a message object of type '<ReadRobotIO-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'io_type))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'io_type))
  (cl:let* ((signed (cl:slot-value msg 'io_index)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReadRobotIO-request>) istream)
  "Deserializes a message object of type '<ReadRobotIO-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'io_type) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'io_type) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'io_index) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReadRobotIO-request>)))
  "Returns string type for a service object of type '<ReadRobotIO-request>"
  "demo_interface/ReadRobotIORequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReadRobotIO-request)))
  "Returns string type for a service object of type 'ReadRobotIO-request"
  "demo_interface/ReadRobotIORequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReadRobotIO-request>)))
  "Returns md5sum for a message object of type '<ReadRobotIO-request>"
  "5d10bf6c42285a91d7053eb3125941e8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReadRobotIO-request)))
  "Returns md5sum for a message object of type 'ReadRobotIO-request"
  "5d10bf6c42285a91d7053eb3125941e8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReadRobotIO-request>)))
  "Returns full string definition for message of type '<ReadRobotIO-request>"
  (cl:format cl:nil "string io_type~%int32 io_index~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReadRobotIO-request)))
  "Returns full string definition for message of type 'ReadRobotIO-request"
  (cl:format cl:nil "string io_type~%int32 io_index~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReadRobotIO-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'io_type))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReadRobotIO-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ReadRobotIO-request
    (cl:cons ':io_type (io_type msg))
    (cl:cons ':io_index (io_index msg))
))
;//! \htmlinclude ReadRobotIO-response.msg.html

(cl:defclass <ReadRobotIO-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (value
    :reader value
    :initarg :value
    :type cl:float
    :initform 0.0)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform ""))
)

(cl:defclass ReadRobotIO-response (<ReadRobotIO-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReadRobotIO-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReadRobotIO-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name demo_interface-srv:<ReadRobotIO-response> is deprecated: use demo_interface-srv:ReadRobotIO-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <ReadRobotIO-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader demo_interface-srv:success-val is deprecated.  Use demo_interface-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <ReadRobotIO-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader demo_interface-srv:value-val is deprecated.  Use demo_interface-srv:value instead.")
  (value m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <ReadRobotIO-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader demo_interface-srv:message-val is deprecated.  Use demo_interface-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReadRobotIO-response>) ostream)
  "Serializes a message object of type '<ReadRobotIO-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReadRobotIO-response>) istream)
  "Deserializes a message object of type '<ReadRobotIO-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'value) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReadRobotIO-response>)))
  "Returns string type for a service object of type '<ReadRobotIO-response>"
  "demo_interface/ReadRobotIOResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReadRobotIO-response)))
  "Returns string type for a service object of type 'ReadRobotIO-response"
  "demo_interface/ReadRobotIOResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReadRobotIO-response>)))
  "Returns md5sum for a message object of type '<ReadRobotIO-response>"
  "5d10bf6c42285a91d7053eb3125941e8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReadRobotIO-response)))
  "Returns md5sum for a message object of type 'ReadRobotIO-response"
  "5d10bf6c42285a91d7053eb3125941e8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReadRobotIO-response>)))
  "Returns full string definition for message of type '<ReadRobotIO-response>"
  (cl:format cl:nil "bool success~%float64 value~%string message~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReadRobotIO-response)))
  "Returns full string definition for message of type 'ReadRobotIO-response"
  (cl:format cl:nil "bool success~%float64 value~%string message~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReadRobotIO-response>))
  (cl:+ 0
     1
     8
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReadRobotIO-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ReadRobotIO-response
    (cl:cons ':success (success msg))
    (cl:cons ':value (value msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ReadRobotIO)))
  'ReadRobotIO-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ReadRobotIO)))
  'ReadRobotIO-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReadRobotIO)))
  "Returns string type for a service object of type '<ReadRobotIO>"
  "demo_interface/ReadRobotIO")