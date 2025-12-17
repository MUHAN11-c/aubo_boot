;; Auto-generated. Do not edit!


(when (boundp 'demo_interface::SetRobotEnable)
  (if (not (find-package "DEMO_INTERFACE"))
    (make-package "DEMO_INTERFACE"))
  (shadow 'SetRobotEnable (find-package "DEMO_INTERFACE")))
(unless (find-package "DEMO_INTERFACE::SETROBOTENABLE")
  (make-package "DEMO_INTERFACE::SETROBOTENABLE"))
(unless (find-package "DEMO_INTERFACE::SETROBOTENABLEREQUEST")
  (make-package "DEMO_INTERFACE::SETROBOTENABLEREQUEST"))
(unless (find-package "DEMO_INTERFACE::SETROBOTENABLERESPONSE")
  (make-package "DEMO_INTERFACE::SETROBOTENABLERESPONSE"))

(in-package "ROS")





(defclass demo_interface::SetRobotEnableRequest
  :super ros::object
  :slots (_enable ))

(defmethod demo_interface::SetRobotEnableRequest
  (:init
   (&key
    ((:enable __enable) nil)
    )
   (send-super :init)
   (setq _enable __enable)
   self)
  (:enable
   (&optional (__enable :null))
   (if (not (eq __enable :null)) (setq _enable __enable)) _enable)
  (:serialization-length
   ()
   (+
    ;; bool _enable
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _enable
       (if _enable (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _enable
     (setq _enable (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass demo_interface::SetRobotEnableResponse
  :super ros::object
  :slots (_success _error_code _message ))

(defmethod demo_interface::SetRobotEnableResponse
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

(defclass demo_interface::SetRobotEnable
  :super ros::object
  :slots ())

(setf (get demo_interface::SetRobotEnable :md5sum-) "e0593992fefd6b0d91d0bf002fc60a2a")
(setf (get demo_interface::SetRobotEnable :datatype-) "demo_interface/SetRobotEnable")
(setf (get demo_interface::SetRobotEnable :request) demo_interface::SetRobotEnableRequest)
(setf (get demo_interface::SetRobotEnable :response) demo_interface::SetRobotEnableResponse)

(defmethod demo_interface::SetRobotEnableRequest
  (:response () (instance demo_interface::SetRobotEnableResponse :init)))

(setf (get demo_interface::SetRobotEnableRequest :md5sum-) "e0593992fefd6b0d91d0bf002fc60a2a")
(setf (get demo_interface::SetRobotEnableRequest :datatype-) "demo_interface/SetRobotEnableRequest")
(setf (get demo_interface::SetRobotEnableRequest :definition-)
      "bool enable
---
bool success
int32 error_code
string message


")

(setf (get demo_interface::SetRobotEnableResponse :md5sum-) "e0593992fefd6b0d91d0bf002fc60a2a")
(setf (get demo_interface::SetRobotEnableResponse :datatype-) "demo_interface/SetRobotEnableResponse")
(setf (get demo_interface::SetRobotEnableResponse :definition-)
      "bool enable
---
bool success
int32 error_code
string message


")



(provide :demo_interface/SetRobotEnable "e0593992fefd6b0d91d0bf002fc60a2a")


