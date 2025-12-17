; Auto-generated. Do not edit!


(cl:in-package demo_interface-srv)


;//! \htmlinclude SetRobotEnable-request.msg.html

(cl:defclass <SetRobotEnable-request> (roslisp-msg-protocol:ros-message)
  ((enable
    :reader enable
    :initarg :enable
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetRobotEnable-request (<SetRobotEnable-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetRobotEnable-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetRobotEnable-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name demo_interface-srv:<SetRobotEnable-request> is deprecated: use demo_interface-srv:SetRobotEnable-request instead.")))

(cl:ensure-generic-function 'enable-val :lambda-list '(m))
(cl:defmethod enable-val ((m <SetRobotEnable-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader demo_interface-srv:enable-val is deprecated.  Use demo_interface-srv:enable instead.")
  (enable m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetRobotEnable-request>) ostream)
  "Serializes a message object of type '<SetRobotEnable-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'enable) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetRobotEnable-request>) istream)
  "Deserializes a message object of type '<SetRobotEnable-request>"
    (cl:setf (cl:slot-value msg 'enable) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetRobotEnable-request>)))
  "Returns string type for a service object of type '<SetRobotEnable-request>"
  "demo_interface/SetRobotEnableRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetRobotEnable-request)))
  "Returns string type for a service object of type 'SetRobotEnable-request"
  "demo_interface/SetRobotEnableRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetRobotEnable-request>)))
  "Returns md5sum for a message object of type '<SetRobotEnable-request>"
  "e0593992fefd6b0d91d0bf002fc60a2a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetRobotEnable-request)))
  "Returns md5sum for a message object of type 'SetRobotEnable-request"
  "e0593992fefd6b0d91d0bf002fc60a2a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetRobotEnable-request>)))
  "Returns full string definition for message of type '<SetRobotEnable-request>"
  (cl:format cl:nil "bool enable~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetRobotEnable-request)))
  "Returns full string definition for message of type 'SetRobotEnable-request"
  (cl:format cl:nil "bool enable~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetRobotEnable-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetRobotEnable-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetRobotEnable-request
    (cl:cons ':enable (enable msg))
))
;//! \htmlinclude SetRobotEnable-response.msg.html

(cl:defclass <SetRobotEnable-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass SetRobotEnable-response (<SetRobotEnable-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetRobotEnable-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetRobotEnable-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name demo_interface-srv:<SetRobotEnable-response> is deprecated: use demo_interface-srv:SetRobotEnable-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetRobotEnable-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader demo_interface-srv:success-val is deprecated.  Use demo_interface-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'error_code-val :lambda-list '(m))
(cl:defmethod error_code-val ((m <SetRobotEnable-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader demo_interface-srv:error_code-val is deprecated.  Use demo_interface-srv:error_code instead.")
  (error_code m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <SetRobotEnable-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader demo_interface-srv:message-val is deprecated.  Use demo_interface-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetRobotEnable-response>) ostream)
  "Serializes a message object of type '<SetRobotEnable-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetRobotEnable-response>) istream)
  "Deserializes a message object of type '<SetRobotEnable-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetRobotEnable-response>)))
  "Returns string type for a service object of type '<SetRobotEnable-response>"
  "demo_interface/SetRobotEnableResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetRobotEnable-response)))
  "Returns string type for a service object of type 'SetRobotEnable-response"
  "demo_interface/SetRobotEnableResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetRobotEnable-response>)))
  "Returns md5sum for a message object of type '<SetRobotEnable-response>"
  "e0593992fefd6b0d91d0bf002fc60a2a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetRobotEnable-response)))
  "Returns md5sum for a message object of type 'SetRobotEnable-response"
  "e0593992fefd6b0d91d0bf002fc60a2a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetRobotEnable-response>)))
  "Returns full string definition for message of type '<SetRobotEnable-response>"
  (cl:format cl:nil "bool success~%int32 error_code~%string message~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetRobotEnable-response)))
  "Returns full string definition for message of type 'SetRobotEnable-response"
  (cl:format cl:nil "bool success~%int32 error_code~%string message~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetRobotEnable-response>))
  (cl:+ 0
     1
     4
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetRobotEnable-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetRobotEnable-response
    (cl:cons ':success (success msg))
    (cl:cons ':error_code (error_code msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetRobotEnable)))
  'SetRobotEnable-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetRobotEnable)))
  'SetRobotEnable-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetRobotEnable)))
  "Returns string type for a service object of type '<SetRobotEnable>"
  "demo_interface/SetRobotEnable")