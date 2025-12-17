; Auto-generated. Do not edit!


(cl:in-package demo_interface-srv)


;//! \htmlinclude SetRobotIO-request.msg.html

(cl:defclass <SetRobotIO-request> (roslisp-msg-protocol:ros-message)
  ((io_type
    :reader io_type
    :initarg :io_type
    :type cl:string
    :initform "")
   (io_index
    :reader io_index
    :initarg :io_index
    :type cl:integer
    :initform 0)
   (value
    :reader value
    :initarg :value
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetRobotIO-request (<SetRobotIO-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetRobotIO-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetRobotIO-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name demo_interface-srv:<SetRobotIO-request> is deprecated: use demo_interface-srv:SetRobotIO-request instead.")))

(cl:ensure-generic-function 'io_type-val :lambda-list '(m))
(cl:defmethod io_type-val ((m <SetRobotIO-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader demo_interface-srv:io_type-val is deprecated.  Use demo_interface-srv:io_type instead.")
  (io_type m))

(cl:ensure-generic-function 'io_index-val :lambda-list '(m))
(cl:defmethod io_index-val ((m <SetRobotIO-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader demo_interface-srv:io_index-val is deprecated.  Use demo_interface-srv:io_index instead.")
  (io_index m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <SetRobotIO-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader demo_interface-srv:value-val is deprecated.  Use demo_interface-srv:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetRobotIO-request>) ostream)
  "Serializes a message object of type '<SetRobotIO-request>"
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
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetRobotIO-request>) istream)
  "Deserializes a message object of type '<SetRobotIO-request>"
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetRobotIO-request>)))
  "Returns string type for a service object of type '<SetRobotIO-request>"
  "demo_interface/SetRobotIORequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetRobotIO-request)))
  "Returns string type for a service object of type 'SetRobotIO-request"
  "demo_interface/SetRobotIORequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetRobotIO-request>)))
  "Returns md5sum for a message object of type '<SetRobotIO-request>"
  "2e3250f0a25c75ced7db303a09e99be6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetRobotIO-request)))
  "Returns md5sum for a message object of type 'SetRobotIO-request"
  "2e3250f0a25c75ced7db303a09e99be6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetRobotIO-request>)))
  "Returns full string definition for message of type '<SetRobotIO-request>"
  (cl:format cl:nil "string io_type~%int32 io_index~%float64 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetRobotIO-request)))
  "Returns full string definition for message of type 'SetRobotIO-request"
  (cl:format cl:nil "string io_type~%int32 io_index~%float64 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetRobotIO-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'io_type))
     4
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetRobotIO-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetRobotIO-request
    (cl:cons ':io_type (io_type msg))
    (cl:cons ':io_index (io_index msg))
    (cl:cons ':value (value msg))
))
;//! \htmlinclude SetRobotIO-response.msg.html

(cl:defclass <SetRobotIO-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (error_code
    :reader error_code
    :initarg :error_code
    :type cl:integer
    :initform 0)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform ""))
)

(cl:defclass SetRobotIO-response (<SetRobotIO-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetRobotIO-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetRobotIO-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name demo_interface-srv:<SetRobotIO-response> is deprecated: use demo_interface-srv:SetRobotIO-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetRobotIO-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader demo_interface-srv:success-val is deprecated.  Use demo_interface-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'error_code-val :lambda-list '(m))
(cl:defmethod error_code-val ((m <SetRobotIO-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader demo_interface-srv:error_code-val is deprecated.  Use demo_interface-srv:error_code instead.")
  (error_code m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <SetRobotIO-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader demo_interface-srv:message-val is deprecated.  Use demo_interface-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetRobotIO-response>) ostream)
  "Serializes a message object of type '<SetRobotIO-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'error_code)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetRobotIO-response>) istream)
  "Deserializes a message object of type '<SetRobotIO-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'error_code) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetRobotIO-response>)))
  "Returns string type for a service object of type '<SetRobotIO-response>"
  "demo_interface/SetRobotIOResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetRobotIO-response)))
  "Returns string type for a service object of type 'SetRobotIO-response"
  "demo_interface/SetRobotIOResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetRobotIO-response>)))
  "Returns md5sum for a message object of type '<SetRobotIO-response>"
  "2e3250f0a25c75ced7db303a09e99be6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetRobotIO-response)))
  "Returns md5sum for a message object of type 'SetRobotIO-response"
  "2e3250f0a25c75ced7db303a09e99be6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetRobotIO-response>)))
  "Returns full string definition for message of type '<SetRobotIO-response>"
  (cl:format cl:nil "bool success~%int32 error_code~%string message~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetRobotIO-response)))
  "Returns full string definition for message of type 'SetRobotIO-response"
  (cl:format cl:nil "bool success~%int32 error_code~%string message~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetRobotIO-response>))
  (cl:+ 0
     1
     4
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetRobotIO-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetRobotIO-response
    (cl:cons ':success (success msg))
    (cl:cons ':error_code (error_code msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetRobotIO)))
  'SetRobotIO-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetRobotIO)))
  'SetRobotIO-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetRobotIO)))
  "Returns string type for a service object of type '<SetRobotIO>"
  "demo_interface/SetRobotIO")