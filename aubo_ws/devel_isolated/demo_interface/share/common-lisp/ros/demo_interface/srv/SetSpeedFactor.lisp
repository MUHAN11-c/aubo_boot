; Auto-generated. Do not edit!


(cl:in-package demo_interface-srv)


;//! \htmlinclude SetSpeedFactor-request.msg.html

(cl:defclass <SetSpeedFactor-request> (roslisp-msg-protocol:ros-message)
  ((velocity_factor
    :reader velocity_factor
    :initarg :velocity_factor
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetSpeedFactor-request (<SetSpeedFactor-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetSpeedFactor-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetSpeedFactor-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name demo_interface-srv:<SetSpeedFactor-request> is deprecated: use demo_interface-srv:SetSpeedFactor-request instead.")))

(cl:ensure-generic-function 'velocity_factor-val :lambda-list '(m))
(cl:defmethod velocity_factor-val ((m <SetSpeedFactor-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader demo_interface-srv:velocity_factor-val is deprecated.  Use demo_interface-srv:velocity_factor instead.")
  (velocity_factor m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetSpeedFactor-request>) ostream)
  "Serializes a message object of type '<SetSpeedFactor-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'velocity_factor))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetSpeedFactor-request>) istream)
  "Deserializes a message object of type '<SetSpeedFactor-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'velocity_factor) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetSpeedFactor-request>)))
  "Returns string type for a service object of type '<SetSpeedFactor-request>"
  "demo_interface/SetSpeedFactorRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetSpeedFactor-request)))
  "Returns string type for a service object of type 'SetSpeedFactor-request"
  "demo_interface/SetSpeedFactorRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetSpeedFactor-request>)))
  "Returns md5sum for a message object of type '<SetSpeedFactor-request>"
  "743f72e0c72310f8ab529ef4bb485765")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetSpeedFactor-request)))
  "Returns md5sum for a message object of type 'SetSpeedFactor-request"
  "743f72e0c72310f8ab529ef4bb485765")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetSpeedFactor-request>)))
  "Returns full string definition for message of type '<SetSpeedFactor-request>"
  (cl:format cl:nil "float32 velocity_factor~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetSpeedFactor-request)))
  "Returns full string definition for message of type 'SetSpeedFactor-request"
  (cl:format cl:nil "float32 velocity_factor~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetSpeedFactor-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetSpeedFactor-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetSpeedFactor-request
    (cl:cons ':velocity_factor (velocity_factor msg))
))
;//! \htmlinclude SetSpeedFactor-response.msg.html

(cl:defclass <SetSpeedFactor-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform ""))
)

(cl:defclass SetSpeedFactor-response (<SetSpeedFactor-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetSpeedFactor-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetSpeedFactor-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name demo_interface-srv:<SetSpeedFactor-response> is deprecated: use demo_interface-srv:SetSpeedFactor-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetSpeedFactor-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader demo_interface-srv:success-val is deprecated.  Use demo_interface-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <SetSpeedFactor-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader demo_interface-srv:message-val is deprecated.  Use demo_interface-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetSpeedFactor-response>) ostream)
  "Serializes a message object of type '<SetSpeedFactor-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetSpeedFactor-response>) istream)
  "Deserializes a message object of type '<SetSpeedFactor-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetSpeedFactor-response>)))
  "Returns string type for a service object of type '<SetSpeedFactor-response>"
  "demo_interface/SetSpeedFactorResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetSpeedFactor-response)))
  "Returns string type for a service object of type 'SetSpeedFactor-response"
  "demo_interface/SetSpeedFactorResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetSpeedFactor-response>)))
  "Returns md5sum for a message object of type '<SetSpeedFactor-response>"
  "743f72e0c72310f8ab529ef4bb485765")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetSpeedFactor-response)))
  "Returns md5sum for a message object of type 'SetSpeedFactor-response"
  "743f72e0c72310f8ab529ef4bb485765")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetSpeedFactor-response>)))
  "Returns full string definition for message of type '<SetSpeedFactor-response>"
  (cl:format cl:nil "bool success~%string message~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetSpeedFactor-response)))
  "Returns full string definition for message of type 'SetSpeedFactor-response"
  (cl:format cl:nil "bool success~%string message~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetSpeedFactor-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetSpeedFactor-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetSpeedFactor-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetSpeedFactor)))
  'SetSpeedFactor-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetSpeedFactor)))
  'SetSpeedFactor-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetSpeedFactor)))
  "Returns string type for a service object of type '<SetSpeedFactor>"
  "demo_interface/SetSpeedFactor")