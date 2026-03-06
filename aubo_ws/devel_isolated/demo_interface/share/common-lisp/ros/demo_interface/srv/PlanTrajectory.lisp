; Auto-generated. Do not edit!


(cl:in-package demo_interface-srv)


;//! \htmlinclude PlanTrajectory-request.msg.html

(cl:defclass <PlanTrajectory-request> (roslisp-msg-protocol:ros-message)
  ((target_pose
    :reader target_pose
    :initarg :target_pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (use_joints
    :reader use_joints
    :initarg :use_joints
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass PlanTrajectory-request (<PlanTrajectory-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlanTrajectory-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlanTrajectory-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name demo_interface-srv:<PlanTrajectory-request> is deprecated: use demo_interface-srv:PlanTrajectory-request instead.")))

(cl:ensure-generic-function 'target_pose-val :lambda-list '(m))
(cl:defmethod target_pose-val ((m <PlanTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader demo_interface-srv:target_pose-val is deprecated.  Use demo_interface-srv:target_pose instead.")
  (target_pose m))

(cl:ensure-generic-function 'use_joints-val :lambda-list '(m))
(cl:defmethod use_joints-val ((m <PlanTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader demo_interface-srv:use_joints-val is deprecated.  Use demo_interface-srv:use_joints instead.")
  (use_joints m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlanTrajectory-request>) ostream)
  "Serializes a message object of type '<PlanTrajectory-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'target_pose) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'use_joints) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlanTrajectory-request>) istream)
  "Deserializes a message object of type '<PlanTrajectory-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'target_pose) istream)
    (cl:setf (cl:slot-value msg 'use_joints) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlanTrajectory-request>)))
  "Returns string type for a service object of type '<PlanTrajectory-request>"
  "demo_interface/PlanTrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlanTrajectory-request)))
  "Returns string type for a service object of type 'PlanTrajectory-request"
  "demo_interface/PlanTrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlanTrajectory-request>)))
  "Returns md5sum for a message object of type '<PlanTrajectory-request>"
  "07df15be4b07a5a918366cc9c76a3ccd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlanTrajectory-request)))
  "Returns md5sum for a message object of type 'PlanTrajectory-request"
  "07df15be4b07a5a918366cc9c76a3ccd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlanTrajectory-request>)))
  "Returns full string definition for message of type '<PlanTrajectory-request>"
  (cl:format cl:nil "geometry_msgs/Pose target_pose~%bool use_joints~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlanTrajectory-request)))
  "Returns full string definition for message of type 'PlanTrajectory-request"
  (cl:format cl:nil "geometry_msgs/Pose target_pose~%bool use_joints~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlanTrajectory-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'target_pose))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlanTrajectory-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PlanTrajectory-request
    (cl:cons ':target_pose (target_pose msg))
    (cl:cons ':use_joints (use_joints msg))
))
;//! \htmlinclude PlanTrajectory-response.msg.html

(cl:defclass <PlanTrajectory-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (trajectory
    :reader trajectory
    :initarg :trajectory
    :type trajectory_msgs-msg:JointTrajectory
    :initform (cl:make-instance 'trajectory_msgs-msg:JointTrajectory))
   (planning_time
    :reader planning_time
    :initarg :planning_time
    :type cl:float
    :initform 0.0)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform ""))
)

(cl:defclass PlanTrajectory-response (<PlanTrajectory-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlanTrajectory-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlanTrajectory-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name demo_interface-srv:<PlanTrajectory-response> is deprecated: use demo_interface-srv:PlanTrajectory-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <PlanTrajectory-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader demo_interface-srv:success-val is deprecated.  Use demo_interface-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'trajectory-val :lambda-list '(m))
(cl:defmethod trajectory-val ((m <PlanTrajectory-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader demo_interface-srv:trajectory-val is deprecated.  Use demo_interface-srv:trajectory instead.")
  (trajectory m))

(cl:ensure-generic-function 'planning_time-val :lambda-list '(m))
(cl:defmethod planning_time-val ((m <PlanTrajectory-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader demo_interface-srv:planning_time-val is deprecated.  Use demo_interface-srv:planning_time instead.")
  (planning_time m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <PlanTrajectory-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader demo_interface-srv:message-val is deprecated.  Use demo_interface-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlanTrajectory-response>) ostream)
  "Serializes a message object of type '<PlanTrajectory-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'trajectory) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'planning_time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlanTrajectory-response>) istream)
  "Deserializes a message object of type '<PlanTrajectory-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'trajectory) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'planning_time) (roslisp-utils:decode-single-float-bits bits)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlanTrajectory-response>)))
  "Returns string type for a service object of type '<PlanTrajectory-response>"
  "demo_interface/PlanTrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlanTrajectory-response)))
  "Returns string type for a service object of type 'PlanTrajectory-response"
  "demo_interface/PlanTrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlanTrajectory-response>)))
  "Returns md5sum for a message object of type '<PlanTrajectory-response>"
  "07df15be4b07a5a918366cc9c76a3ccd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlanTrajectory-response)))
  "Returns md5sum for a message object of type 'PlanTrajectory-response"
  "07df15be4b07a5a918366cc9c76a3ccd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlanTrajectory-response>)))
  "Returns full string definition for message of type '<PlanTrajectory-response>"
  (cl:format cl:nil "bool success~%trajectory_msgs/JointTrajectory trajectory~%float32 planning_time~%string message~%~%~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectory~%Header header~%string[] joint_names~%JointTrajectoryPoint[] points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectoryPoint~%# Each trajectory point specifies either positions[, velocities[, accelerations]]~%# or positions[, effort] for the trajectory to be executed.~%# All specified values are in the same order as the joint names in JointTrajectory.msg~%~%float64[] positions~%float64[] velocities~%float64[] accelerations~%float64[] effort~%duration time_from_start~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlanTrajectory-response)))
  "Returns full string definition for message of type 'PlanTrajectory-response"
  (cl:format cl:nil "bool success~%trajectory_msgs/JointTrajectory trajectory~%float32 planning_time~%string message~%~%~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectory~%Header header~%string[] joint_names~%JointTrajectoryPoint[] points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectoryPoint~%# Each trajectory point specifies either positions[, velocities[, accelerations]]~%# or positions[, effort] for the trajectory to be executed.~%# All specified values are in the same order as the joint names in JointTrajectory.msg~%~%float64[] positions~%float64[] velocities~%float64[] accelerations~%float64[] effort~%duration time_from_start~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlanTrajectory-response>))
  (cl:+ 0
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'trajectory))
     4
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlanTrajectory-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PlanTrajectory-response
    (cl:cons ':success (success msg))
    (cl:cons ':trajectory (trajectory msg))
    (cl:cons ':planning_time (planning_time msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PlanTrajectory)))
  'PlanTrajectory-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PlanTrajectory)))
  'PlanTrajectory-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlanTrajectory)))
  "Returns string type for a service object of type '<PlanTrajectory>"
  "demo_interface/PlanTrajectory")