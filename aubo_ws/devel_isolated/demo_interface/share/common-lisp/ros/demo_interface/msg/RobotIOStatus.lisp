; Auto-generated. Do not edit!


(cl:in-package demo_interface-msg)


;//! \htmlinclude RobotIOStatus.msg.html

(cl:defclass <RobotIOStatus> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (digital_inputs
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
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (tool_io_status
    :reader tool_io_status
    :initarg :tool_io_status
    :type demo_interface-msg:ToolIOStatus
    :initform (cl:make-instance 'demo_interface-msg:ToolIOStatus))
   (is_connected
    :reader is_connected
    :initarg :is_connected
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass RobotIOStatus (<RobotIOStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RobotIOStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RobotIOStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name demo_interface-msg:<RobotIOStatus> is deprecated: use demo_interface-msg:RobotIOStatus instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <RobotIOStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader demo_interface-msg:header-val is deprecated.  Use demo_interface-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'digital_inputs-val :lambda-list '(m))
(cl:defmethod digital_inputs-val ((m <RobotIOStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader demo_interface-msg:digital_inputs-val is deprecated.  Use demo_interface-msg:digital_inputs instead.")
  (digital_inputs m))

(cl:ensure-generic-function 'digital_outputs-val :lambda-list '(m))
(cl:defmethod digital_outputs-val ((m <RobotIOStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader demo_interface-msg:digital_outputs-val is deprecated.  Use demo_interface-msg:digital_outputs instead.")
  (digital_outputs m))

(cl:ensure-generic-function 'analog_inputs-val :lambda-list '(m))
(cl:defmethod analog_inputs-val ((m <RobotIOStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader demo_interface-msg:analog_inputs-val is deprecated.  Use demo_interface-msg:analog_inputs instead.")
  (analog_inputs m))

(cl:ensure-generic-function 'analog_outputs-val :lambda-list '(m))
(cl:defmethod analog_outputs-val ((m <RobotIOStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader demo_interface-msg:analog_outputs-val is deprecated.  Use demo_interface-msg:analog_outputs instead.")
  (analog_outputs m))

(cl:ensure-generic-function 'tool_io_status-val :lambda-list '(m))
(cl:defmethod tool_io_status-val ((m <RobotIOStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader demo_interface-msg:tool_io_status-val is deprecated.  Use demo_interface-msg:tool_io_status instead.")
  (tool_io_status m))

(cl:ensure-generic-function 'is_connected-val :lambda-list '(m))
(cl:defmethod is_connected-val ((m <RobotIOStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader demo_interface-msg:is_connected-val is deprecated.  Use demo_interface-msg:is_connected instead.")
  (is_connected m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RobotIOStatus>) ostream)
  "Serializes a message object of type '<RobotIOStatus>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
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
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'tool_io_status) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_connected) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RobotIOStatus>) istream)
  "Deserializes a message object of type '<RobotIOStatus>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
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
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'tool_io_status) istream)
    (cl:setf (cl:slot-value msg 'is_connected) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RobotIOStatus>)))
  "Returns string type for a message object of type '<RobotIOStatus>"
  "demo_interface/RobotIOStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotIOStatus)))
  "Returns string type for a message object of type 'RobotIOStatus"
  "demo_interface/RobotIOStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RobotIOStatus>)))
  "Returns md5sum for a message object of type '<RobotIOStatus>"
  "241d2a7d7ac2df6c1d3ad9602be6f25a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RobotIOStatus)))
  "Returns md5sum for a message object of type 'RobotIOStatus"
  "241d2a7d7ac2df6c1d3ad9602be6f25a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RobotIOStatus>)))
  "Returns full string definition for message of type '<RobotIOStatus>"
  (cl:format cl:nil "std_msgs/Header header~%bool[] digital_inputs~%bool[] digital_outputs~%float32[] analog_inputs~%float32[] analog_outputs~%ToolIOStatus tool_io_status~%bool is_connected~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: demo_interface/ToolIOStatus~%bool[] digital_inputs~%bool[] digital_outputs~%float32[] analog_inputs~%float32[] analog_outputs~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RobotIOStatus)))
  "Returns full string definition for message of type 'RobotIOStatus"
  (cl:format cl:nil "std_msgs/Header header~%bool[] digital_inputs~%bool[] digital_outputs~%float32[] analog_inputs~%float32[] analog_outputs~%ToolIOStatus tool_io_status~%bool is_connected~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: demo_interface/ToolIOStatus~%bool[] digital_inputs~%bool[] digital_outputs~%float32[] analog_inputs~%float32[] analog_outputs~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RobotIOStatus>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'digital_inputs) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'digital_outputs) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'analog_inputs) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'analog_outputs) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'tool_io_status))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RobotIOStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'RobotIOStatus
    (cl:cons ':header (header msg))
    (cl:cons ':digital_inputs (digital_inputs msg))
    (cl:cons ':digital_outputs (digital_outputs msg))
    (cl:cons ':analog_inputs (analog_inputs msg))
    (cl:cons ':analog_outputs (analog_outputs msg))
    (cl:cons ':tool_io_status (tool_io_status msg))
    (cl:cons ':is_connected (is_connected msg))
))
