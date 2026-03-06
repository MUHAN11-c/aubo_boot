; Auto-generated. Do not edit!


(cl:in-package demo_interface-srv)


;//! \htmlinclude MoveToPose-request.msg.html

(cl:defclass <MoveToPose-request> (roslisp-msg-protocol:ros-message)
  ((target_pose
    :reader target_pose
    :initarg :target_pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (use_joints
    :reader use_joints
    :initarg :use_joints
    :type cl:boolean
    :initform cl:nil)
   (velocity_factor
    :reader velocity_factor
    :initarg :velocity_factor
    :type cl:float
    :initform 0.0)
   (acceleration_factor
    :reader acceleration_factor
    :initarg :acceleration_factor
    :type cl:float
    :initform 0.0))
)

(cl:defclass MoveToPose-request (<MoveToPose-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveToPose-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveToPose-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name demo_interface-srv:<MoveToPose-request> is deprecated: use demo_interface-srv:MoveToPose-request instead.")))

(cl:ensure-generic-function 'target_pose-val :lambda-list '(m))
(cl:defmethod target_pose-val ((m <MoveToPose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader demo_interface-srv:target_pose-val is deprecated.  Use demo_interface-srv:target_pose instead.")
  (target_pose m))

(cl:ensure-generic-function 'use_joints-val :lambda-list '(m))
(cl:defmethod use_joints-val ((m <MoveToPose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader demo_interface-srv:use_joints-val is deprecated.  Use demo_interface-srv:use_joints instead.")
  (use_joints m))

(cl:ensure-generic-function 'velocity_factor-val :lambda-list '(m))
(cl:defmethod velocity_factor-val ((m <MoveToPose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader demo_interface-srv:velocity_factor-val is deprecated.  Use demo_interface-srv:velocity_factor instead.")
  (velocity_factor m))

(cl:ensure-generic-function 'acceleration_factor-val :lambda-list '(m))
(cl:defmethod acceleration_factor-val ((m <MoveToPose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader demo_interface-srv:acceleration_factor-val is deprecated.  Use demo_interface-srv:acceleration_factor instead.")
  (acceleration_factor m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveToPose-request>) ostream)
  "Serializes a message object of type '<MoveToPose-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'target_pose) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'use_joints) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'velocity_factor))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'acceleration_factor))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveToPose-request>) istream)
  "Deserializes a message object of type '<MoveToPose-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'target_pose) istream)
    (cl:setf (cl:slot-value msg 'use_joints) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'velocity_factor) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'acceleration_factor) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveToPose-request>)))
  "Returns string type for a service object of type '<MoveToPose-request>"
  "demo_interface/MoveToPoseRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveToPose-request)))
  "Returns string type for a service object of type 'MoveToPose-request"
  "demo_interface/MoveToPoseRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveToPose-request>)))
  "Returns md5sum for a message object of type '<MoveToPose-request>"
  "13d651572c46e71643945d975979b6fc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveToPose-request)))
  "Returns md5sum for a message object of type 'MoveToPose-request"
  "13d651572c46e71643945d975979b6fc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveToPose-request>)))
  "Returns full string definition for message of type '<MoveToPose-request>"
  (cl:format cl:nil "geometry_msgs/Pose target_pose~%bool use_joints~%float32 velocity_factor~%float32 acceleration_factor~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveToPose-request)))
  "Returns full string definition for message of type 'MoveToPose-request"
  (cl:format cl:nil "geometry_msgs/Pose target_pose~%bool use_joints~%float32 velocity_factor~%float32 acceleration_factor~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveToPose-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'target_pose))
     1
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveToPose-request>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveToPose-request
    (cl:cons ':target_pose (target_pose msg))
    (cl:cons ':use_joints (use_joints msg))
    (cl:cons ':velocity_factor (velocity_factor msg))
    (cl:cons ':acceleration_factor (acceleration_factor msg))
))
;//! \htmlinclude MoveToPose-response.msg.html

(cl:defclass <MoveToPose-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass MoveToPose-response (<MoveToPose-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveToPose-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveToPose-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name demo_interface-srv:<MoveToPose-response> is deprecated: use demo_interface-srv:MoveToPose-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <MoveToPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader demo_interface-srv:success-val is deprecated.  Use demo_interface-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'error_code-val :lambda-list '(m))
(cl:defmethod error_code-val ((m <MoveToPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader demo_interface-srv:error_code-val is deprecated.  Use demo_interface-srv:error_code instead.")
  (error_code m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <MoveToPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader demo_interface-srv:message-val is deprecated.  Use demo_interface-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveToPose-response>) ostream)
  "Serializes a message object of type '<MoveToPose-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveToPose-response>) istream)
  "Deserializes a message object of type '<MoveToPose-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveToPose-response>)))
  "Returns string type for a service object of type '<MoveToPose-response>"
  "demo_interface/MoveToPoseResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveToPose-response)))
  "Returns string type for a service object of type 'MoveToPose-response"
  "demo_interface/MoveToPoseResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveToPose-response>)))
  "Returns md5sum for a message object of type '<MoveToPose-response>"
  "13d651572c46e71643945d975979b6fc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveToPose-response)))
  "Returns md5sum for a message object of type 'MoveToPose-response"
  "13d651572c46e71643945d975979b6fc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveToPose-response>)))
  "Returns full string definition for message of type '<MoveToPose-response>"
  (cl:format cl:nil "bool success~%int32 error_code~%string message~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveToPose-response)))
  "Returns full string definition for message of type 'MoveToPose-response"
  (cl:format cl:nil "bool success~%int32 error_code~%string message~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveToPose-response>))
  (cl:+ 0
     1
     4
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveToPose-response>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveToPose-response
    (cl:cons ':success (success msg))
    (cl:cons ':error_code (error_code msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'MoveToPose)))
  'MoveToPose-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'MoveToPose)))
  'MoveToPose-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveToPose)))
  "Returns string type for a service object of type '<MoveToPose>"
  "demo_interface/MoveToPose")