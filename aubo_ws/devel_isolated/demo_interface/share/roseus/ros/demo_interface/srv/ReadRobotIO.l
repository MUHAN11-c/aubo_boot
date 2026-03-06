;; Auto-generated. Do not edit!


(when (boundp 'demo_interface::ReadRobotIO)
  (if (not (find-package "DEMO_INTERFACE"))
    (make-package "DEMO_INTERFACE"))
  (shadow 'ReadRobotIO (find-package "DEMO_INTERFACE")))
(unless (find-package "DEMO_INTERFACE::READROBOTIO")
  (make-package "DEMO_INTERFACE::READROBOTIO"))
(unless (find-package "DEMO_INTERFACE::READROBOTIOREQUEST")
  (make-package "DEMO_INTERFACE::READROBOTIOREQUEST"))
(unless (find-package "DEMO_INTERFACE::READROBOTIORESPONSE")
  (make-package "DEMO_INTERFACE::READROBOTIORESPONSE"))

(in-package "ROS")





(defclass demo_interface::ReadRobotIORequest
  :super ros::object
  :slots (_io_type _io_index ))

(defmethod demo_interface::ReadRobotIORequest
  (:init
   (&key
    ((:io_type __io_type) "")
    ((:io_index __io_index) 0)
    )
   (send-super :init)
   (setq _io_type (string __io_type))
   (setq _io_index (round __io_index))
   self)
  (:io_type
   (&optional __io_type)
   (if __io_type (setq _io_type __io_type)) _io_type)
  (:io_index
   (&optional __io_index)
   (if __io_index (setq _io_index __io_index)) _io_index)
  (:serialization-length
   ()
   (+
    ;; string _io_type
    4 (length _io_type)
    ;; int32 _io_index
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _io_type
       (write-long (length _io_type) s) (princ _io_type s)
     ;; int32 _io_index
       (write-long _io_index s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _io_type
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _io_type (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _io_index
     (setq _io_index (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass demo_interface::ReadRobotIOResponse
  :super ros::object
  :slots (_success _value _message ))

(defmethod demo_interface::ReadRobotIOResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:value __value) 0.0)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _value (float __value))
   (setq _message (string __message))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; float64 _value
    8
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; float64 _value
       (sys::poke _value (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64 _value
     (setq _value (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass demo_interface::ReadRobotIO
  :super ros::object
  :slots ())

(setf (get demo_interface::ReadRobotIO :md5sum-) "5d10bf6c42285a91d7053eb3125941e8")
(setf (get demo_interface::ReadRobotIO :datatype-) "demo_interface/ReadRobotIO")
(setf (get demo_interface::ReadRobotIO :request) demo_interface::ReadRobotIORequest)
(setf (get demo_interface::ReadRobotIO :response) demo_interface::ReadRobotIOResponse)

(defmethod demo_interface::ReadRobotIORequest
  (:response () (instance demo_interface::ReadRobotIOResponse :init)))

(setf (get demo_interface::ReadRobotIORequest :md5sum-) "5d10bf6c42285a91d7053eb3125941e8")
(setf (get demo_interface::ReadRobotIORequest :datatype-) "demo_interface/ReadRobotIORequest")
(setf (get demo_interface::ReadRobotIORequest :definition-)
      "string io_type
int32 io_index
---
bool success
float64 value
string message


")

(setf (get demo_interface::ReadRobotIOResponse :md5sum-) "5d10bf6c42285a91d7053eb3125941e8")
(setf (get demo_interface::ReadRobotIOResponse :datatype-) "demo_interface/ReadRobotIOResponse")
(setf (get demo_interface::ReadRobotIOResponse :definition-)
      "string io_type
int32 io_index
---
bool success
float64 value
string message


")



(provide :demo_interface/ReadRobotIO "5d10bf6c42285a91d7053eb3125941e8")


