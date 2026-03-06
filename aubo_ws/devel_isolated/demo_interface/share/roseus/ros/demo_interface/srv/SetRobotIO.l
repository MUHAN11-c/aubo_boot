;; Auto-generated. Do not edit!


(when (boundp 'demo_interface::SetRobotIO)
  (if (not (find-package "DEMO_INTERFACE"))
    (make-package "DEMO_INTERFACE"))
  (shadow 'SetRobotIO (find-package "DEMO_INTERFACE")))
(unless (find-package "DEMO_INTERFACE::SETROBOTIO")
  (make-package "DEMO_INTERFACE::SETROBOTIO"))
(unless (find-package "DEMO_INTERFACE::SETROBOTIOREQUEST")
  (make-package "DEMO_INTERFACE::SETROBOTIOREQUEST"))
(unless (find-package "DEMO_INTERFACE::SETROBOTIORESPONSE")
  (make-package "DEMO_INTERFACE::SETROBOTIORESPONSE"))

(in-package "ROS")





(defclass demo_interface::SetRobotIORequest
  :super ros::object
  :slots (_io_type _io_index _value ))

(defmethod demo_interface::SetRobotIORequest
  (:init
   (&key
    ((:io_type __io_type) "")
    ((:io_index __io_index) 0)
    ((:value __value) 0.0)
    )
   (send-super :init)
   (setq _io_type (string __io_type))
   (setq _io_index (round __io_index))
   (setq _value (float __value))
   self)
  (:io_type
   (&optional __io_type)
   (if __io_type (setq _io_type __io_type)) _io_type)
  (:io_index
   (&optional __io_index)
   (if __io_index (setq _io_index __io_index)) _io_index)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:serialization-length
   ()
   (+
    ;; string _io_type
    4 (length _io_type)
    ;; int32 _io_index
    4
    ;; float64 _value
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _io_type
       (write-long (length _io_type) s) (princ _io_type s)
     ;; int32 _io_index
       (write-long _io_index s)
     ;; float64 _value
       (sys::poke _value (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _io_type
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _io_type (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _io_index
     (setq _io_index (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _value
     (setq _value (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass demo_interface::SetRobotIOResponse
  :super ros::object
  :slots (_success _error_code _message ))

(defmethod demo_interface::SetRobotIOResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:error_code __error_code) 0)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _error_code (round __error_code))
   (setq _message (string __message))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:error_code
   (&optional __error_code)
   (if __error_code (setq _error_code __error_code)) _error_code)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; int32 _error_code
    4
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; int32 _error_code
       (write-long _error_code s)
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int32 _error_code
     (setq _error_code (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass demo_interface::SetRobotIO
  :super ros::object
  :slots ())

(setf (get demo_interface::SetRobotIO :md5sum-) "2e3250f0a25c75ced7db303a09e99be6")
(setf (get demo_interface::SetRobotIO :datatype-) "demo_interface/SetRobotIO")
(setf (get demo_interface::SetRobotIO :request) demo_interface::SetRobotIORequest)
(setf (get demo_interface::SetRobotIO :response) demo_interface::SetRobotIOResponse)

(defmethod demo_interface::SetRobotIORequest
  (:response () (instance demo_interface::SetRobotIOResponse :init)))

(setf (get demo_interface::SetRobotIORequest :md5sum-) "2e3250f0a25c75ced7db303a09e99be6")
(setf (get demo_interface::SetRobotIORequest :datatype-) "demo_interface/SetRobotIORequest")
(setf (get demo_interface::SetRobotIORequest :definition-)
      "string io_type
int32 io_index
float64 value
---
bool success
int32 error_code
string message


")

(setf (get demo_interface::SetRobotIOResponse :md5sum-) "2e3250f0a25c75ced7db303a09e99be6")
(setf (get demo_interface::SetRobotIOResponse :datatype-) "demo_interface/SetRobotIOResponse")
(setf (get demo_interface::SetRobotIOResponse :definition-)
      "string io_type
int32 io_index
float64 value
---
bool success
int32 error_code
string message


")



(provide :demo_interface/SetRobotIO "2e3250f0a25c75ced7db303a09e99be6")


