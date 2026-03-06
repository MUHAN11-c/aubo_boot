; Auto-generated. Do not edit!


(cl:in-package rviz-srv)


;//! \htmlinclude SendFilePath-request.msg.html

(cl:defclass <SendFilePath-request> (roslisp-msg-protocol:ros-message)
  ((path
    :reader path
    :initarg :path
    :type std_msgs-msg:String
    :initform (cl:make-instance 'std_msgs-msg:String)))
)

(cl:defclass SendFilePath-request (<SendFilePath-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SendFilePath-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SendFilePath-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rviz-srv:<SendFilePath-request> is deprecated: use rviz-srv:SendFilePath-request instead.")))

(cl:ensure-generic-function 'path-val :lambda-list '(m))
(cl:defmethod path-val ((m <SendFilePath-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rviz-srv:path-val is deprecated.  Use rviz-srv:path instead.")
  (path m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SendFilePath-request>) ostream)
  "Serializes a message object of type '<SendFilePath-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'path) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SendFilePath-request>) istream)
  "Deserializes a message object of type '<SendFilePath-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'path) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SendFilePath-request>)))
  "Returns string type for a service object of type '<SendFilePath-request>"
  "rviz/SendFilePathRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SendFilePath-request)))
  "Returns string type for a service object of type 'SendFilePath-request"
  "rviz/SendFilePathRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SendFilePath-request>)))
  "Returns md5sum for a message object of type '<SendFilePath-request>"
  "92f17244ff295b327b62c307fafe751b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SendFilePath-request)))
  "Returns md5sum for a message object of type 'SendFilePath-request"
  "92f17244ff295b327b62c307fafe751b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SendFilePath-request>)))
  "Returns full string definition for message of type '<SendFilePath-request>"
  (cl:format cl:nil "std_msgs/String path # absolute path to file or directory~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SendFilePath-request)))
  "Returns full string definition for message of type 'SendFilePath-request"
  (cl:format cl:nil "std_msgs/String path # absolute path to file or directory~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SendFilePath-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'path))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SendFilePath-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SendFilePath-request
    (cl:cons ':path (path msg))
))
;//! \htmlinclude SendFilePath-response.msg.html

(cl:defclass <SendFilePath-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SendFilePath-response (<SendFilePath-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SendFilePath-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SendFilePath-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rviz-srv:<SendFilePath-response> is deprecated: use rviz-srv:SendFilePath-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SendFilePath-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rviz-srv:success-val is deprecated.  Use rviz-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SendFilePath-response>) ostream)
  "Serializes a message object of type '<SendFilePath-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SendFilePath-response>) istream)
  "Deserializes a message object of type '<SendFilePath-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SendFilePath-response>)))
  "Returns string type for a service object of type '<SendFilePath-response>"
  "rviz/SendFilePathResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SendFilePath-response)))
  "Returns string type for a service object of type 'SendFilePath-response"
  "rviz/SendFilePathResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SendFilePath-response>)))
  "Returns md5sum for a message object of type '<SendFilePath-response>"
  "92f17244ff295b327b62c307fafe751b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SendFilePath-response)))
  "Returns md5sum for a message object of type 'SendFilePath-response"
  "92f17244ff295b327b62c307fafe751b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SendFilePath-response>)))
  "Returns full string definition for message of type '<SendFilePath-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SendFilePath-response)))
  "Returns full string definition for message of type 'SendFilePath-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SendFilePath-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SendFilePath-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SendFilePath-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SendFilePath)))
  'SendFilePath-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SendFilePath)))
  'SendFilePath-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SendFilePath)))
  "Returns string type for a service object of type '<SendFilePath>"
  "rviz/SendFilePath")