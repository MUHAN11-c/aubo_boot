;; Auto-generated. Do not edit!


(when (boundp 'rviz::SendFilePath)
  (if (not (find-package "RVIZ"))
    (make-package "RVIZ"))
  (shadow 'SendFilePath (find-package "RVIZ")))
(unless (find-package "RVIZ::SENDFILEPATH")
  (make-package "RVIZ::SENDFILEPATH"))
(unless (find-package "RVIZ::SENDFILEPATHREQUEST")
  (make-package "RVIZ::SENDFILEPATHREQUEST"))
(unless (find-package "RVIZ::SENDFILEPATHRESPONSE")
  (make-package "RVIZ::SENDFILEPATHRESPONSE"))

(in-package "ROS")

(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))




(defclass rviz::SendFilePathRequest
  :super ros::object
  :slots (_path ))

(defmethod rviz::SendFilePathRequest
  (:init
   (&key
    ((:path __path) (instance std_msgs::String :init))
    )
   (send-super :init)
   (setq _path __path)
   self)
  (:path
   (&rest __path)
   (if (keywordp (car __path))
       (send* _path __path)
     (progn
       (if __path (setq _path (car __path)))
       _path)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/String _path
    (send _path :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/String _path
       (send _path :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/String _path
     (send _path :deserialize buf ptr-) (incf ptr- (send _path :serialization-length))
   ;;
   self)
  )

(defclass rviz::SendFilePathResponse
  :super ros::object
  :slots (_success ))

(defmethod rviz::SendFilePathResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass rviz::SendFilePath
  :super ros::object
  :slots ())

(setf (get rviz::SendFilePath :md5sum-) "92f17244ff295b327b62c307fafe751b")
(setf (get rviz::SendFilePath :datatype-) "rviz/SendFilePath")
(setf (get rviz::SendFilePath :request) rviz::SendFilePathRequest)
(setf (get rviz::SendFilePath :response) rviz::SendFilePathResponse)

(defmethod rviz::SendFilePathRequest
  (:response () (instance rviz::SendFilePathResponse :init)))

(setf (get rviz::SendFilePathRequest :md5sum-) "92f17244ff295b327b62c307fafe751b")
(setf (get rviz::SendFilePathRequest :datatype-) "rviz/SendFilePathRequest")
(setf (get rviz::SendFilePathRequest :definition-)
      "std_msgs/String path # absolute path to file or directory

================================================================================
MSG: std_msgs/String
string data
---
bool success

")

(setf (get rviz::SendFilePathResponse :md5sum-) "92f17244ff295b327b62c307fafe751b")
(setf (get rviz::SendFilePathResponse :datatype-) "rviz/SendFilePathResponse")
(setf (get rviz::SendFilePathResponse :definition-)
      "std_msgs/String path # absolute path to file or directory

================================================================================
MSG: std_msgs/String
string data
---
bool success

")



(provide :rviz/SendFilePath "92f17244ff295b327b62c307fafe751b")


