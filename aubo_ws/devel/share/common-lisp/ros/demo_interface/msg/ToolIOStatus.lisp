; Auto-generated. Do not edit!


(cl:in-package demo_interface-msg)


;//! \htmlinclude ToolIOStatus.msg.html

(cl:defclass <ToolIOStatus> (roslisp-msg-protocol:ros-message)
  ((digital_inputs
    :reader digital_inputs
    :initarg :digital_inputs
    :type (cl:vector cl:boolean)
   :initform (cl:make-array 0 :element-type 'cl:boolean :initial-element cl:nil))
   (digital_outputs
    :reader digital_outputs
    :initarg :digital_outputs
    :type (cl:vector cl:boolean)
   :initform (cl:make-array 0 :element-type 'cl:boolean :initial-element cl:nil))
   (analog_inputs
    :reader analog_inputs
    :initarg :analog_inputs
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (analog_outputs
    :reader analog_outputs
    :initarg :analog_outputs
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass ToolIOStatus (<ToolIOStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ToolIOStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ToolIOStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name demo_interface-msg:<ToolIOStatus> is deprecated: use demo_interface-msg:ToolIOStatus instead.")))

(cl:ensure-generic-function 'digital_inputs-val :lambda-list '(m))
(cl:defmethod digital_inputs-val ((m <ToolIOStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader demo_interface-msg:digital_inputs-val is deprecated.  Use demo_interface-msg:digital_inputs instead.")
  (digital_inputs m))

(cl:ensure-generic-function 'digital_outputs-val :lambda-list '(m))
(cl:defmethod digital_outputs-val ((m <ToolIOStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader demo_interface-msg:digital_outputs-val is deprecated.  Use demo_interface-msg:digital_outputs instead.")
  (digital_outputs m))

(cl:ensure-generic-function 'analog_inputs-val :lambda-list '(m))
(cl:defmethod analog_inputs-val ((m <ToolIOStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader demo_interface-msg:analog_inputs-val is deprecated.  Use demo_interface-msg:analog_inputs instead.")
  (analog_inputs m))

(cl:ensure-generic-function 'analog_outputs-val :lambda-list '(m))
(cl:defmethod analog_outputs-val ((m <ToolIOStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader demo_interface-msg:analog_outputs-val is deprecated.  Use demo_interface-msg:analog_outputs instead.")
  (analog_outputs m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ToolIOStatus>) ostream)
  "Serializes a message object of type '<ToolIOStatus>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'digital_inputs))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if ele 1 0)) ostream))
   (cl:slot-value msg 'digital_inputs))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'digital_outputs))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if ele 1 0)) ostream))
   (cl:slot-value msg 'digital_outputs))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'analog_inputs))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'analog_inputs))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'analog_outputs))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'analog_outputs))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ToolIOStatus>) istream)
  "Deserializes a message object of type '<ToolIOStatus>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'digital_inputs) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'digital_inputs)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:not (cl:zerop (cl:read-byte istream)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'digital_outputs) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'digital_outputs)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:not (cl:zerop (cl:read-byte istream)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'analog_inputs) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'analog_inputs)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'analog_outputs) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'analog_outputs)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ToolIOStatus>)))
  "Returns string type for a message object of type '<ToolIOStatus>"
  "demo_interface/ToolIOStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ToolIOStatus)))
  "Returns string type for a message object of type 'ToolIOStatus"
  "demo_interface/ToolIOStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ToolIOStatus>)))
  "Returns md5sum for a message object of type '<ToolIOStatus>"
  "219be75bd56b8ebdca0dfa526c3803be")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ToolIOStatus)))
  "Returns md5sum for a message object of type 'ToolIOStatus"
  "219be75bd56b8ebdca0dfa526c3803be")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ToolIOStatus>)))
  "Returns full string definition for message of type '<ToolIOStatus>"
  (cl:format cl:nil "bool[] digital_inputs~%bool[] digital_outputs~%float32[] analog_inputs~%float32[] analog_outputs~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ToolIOStatus)))
  "Returns full string definition for message of type 'ToolIOStatus"
  (cl:format cl:nil "bool[] digital_inputs~%bool[] digital_outputs~%float32[] analog_inputs~%float32[] analog_outputs~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ToolIOStatus>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'digital_inputs) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'digital_outputs) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'analog_inputs) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'analog_outputs) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ToolIOStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'ToolIOStatus
    (cl:cons ':digital_inputs (digital_inputs msg))
    (cl:cons ':digital_outputs (digital_outputs msg))
    (cl:cons ':analog_inputs (analog_inputs msg))
    (cl:cons ':analog_outputs (analog_outputs msg))
))
