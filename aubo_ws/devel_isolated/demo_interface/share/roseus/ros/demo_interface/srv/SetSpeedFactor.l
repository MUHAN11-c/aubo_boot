;; Auto-generated. Do not edit!


(when (boundp 'demo_interface::SetSpeedFactor)
  (if (not (find-package "DEMO_INTERFACE"))
    (make-package "DEMO_INTERFACE"))
  (shadow 'SetSpeedFactor (find-package "DEMO_INTERFACE")))
(unless (find-package "DEMO_INTERFACE::SETSPEEDFACTOR")
  (make-package "DEMO_INTERFACE::SETSPEEDFACTOR"))
(unless (find-package "DEMO_INTERFACE::SETSPEEDFACTORREQUEST")
  (make-package "DEMO_INTERFACE::SETSPEEDFACTORREQUEST"))
(unless (find-package "DEMO_INTERFACE::SETSPEEDFACTORRESPONSE")
  (make-package "DEMO_INTERFACE::SETSPEEDFACTORRESPONSE"))

(in-package "ROS")





(defclass demo_interface::SetSpeedFactorRequest
  :super ros::object
  :slots (_velocity_factor ))

(defmethod demo_interface::SetSpeedFactorRequest
  (:init
   (&key
    ((:velocity_factor __velocity_factor) 0.0)
    )
   (send-super :init)
   (setq _velocity_factor (float __velocity_factor))
   self)
  (:velocity_factor
   (&optional __velocity_factor)
   (if __velocity_factor (setq _velocity_factor __velocity_factor)) _velocity_factor)
  (:serialization-length
   ()
   (+
    ;; float32 _velocity_factor
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _velocity_factor
       (sys::poke _velocity_factor (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _velocity_factor
     (setq _velocity_factor (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass demo_interface::SetSpeedFactorResponse
  :super ros::object
  :slots (_success _message ))

(defmethod demo_interface::SetSpeedFactorResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _message (string __message))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass demo_interface::SetSpeedFactor
  :super ros::object
  :slots ())

(setf (get demo_interface::SetSpeedFactor :md5sum-) "743f72e0c72310f8ab529ef4bb485765")
(setf (get demo_interface::SetSpeedFactor :datatype-) "demo_interface/SetSpeedFactor")
(setf (get demo_interface::SetSpeedFactor :request) demo_interface::SetSpeedFactorRequest)
(setf (get demo_interface::SetSpeedFactor :response) demo_interface::SetSpeedFactorResponse)

(defmethod demo_interface::SetSpeedFactorRequest
  (:response () (instance demo_interface::SetSpeedFactorResponse :init)))

(setf (get demo_interface::SetSpeedFactorRequest :md5sum-) "743f72e0c72310f8ab529ef4bb485765")
(setf (get demo_interface::SetSpeedFactorRequest :datatype-) "demo_interface/SetSpeedFactorRequest")
(setf (get demo_interface::SetSpeedFactorRequest :definition-)
      "float32 velocity_factor
---
bool success
string message


")

(setf (get demo_interface::SetSpeedFactorResponse :md5sum-) "743f72e0c72310f8ab529ef4bb485765")
(setf (get demo_interface::SetSpeedFactorResponse :datatype-) "demo_interface/SetSpeedFactorResponse")
(setf (get demo_interface::SetSpeedFactorResponse :definition-)
      "float32 velocity_factor
---
bool success
string message


")



(provide :demo_interface/SetSpeedFactor "743f72e0c72310f8ab529ef4bb485765")


