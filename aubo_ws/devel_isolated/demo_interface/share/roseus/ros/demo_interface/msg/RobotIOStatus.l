;; Auto-generated. Do not edit!


(when (boundp 'demo_interface::RobotIOStatus)
  (if (not (find-package "DEMO_INTERFACE"))
    (make-package "DEMO_INTERFACE"))
  (shadow 'RobotIOStatus (find-package "DEMO_INTERFACE")))
(unless (find-package "DEMO_INTERFACE::ROBOTIOSTATUS")
  (make-package "DEMO_INTERFACE::ROBOTIOSTATUS"))

(in-package "ROS")
;;//! \htmlinclude RobotIOStatus.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass demo_interface::RobotIOStatus
  :super ros::object
  :slots (_header _digital_inputs _digital_outputs _analog_inputs _analog_outputs _tool_io_status _is_connected ))

(defmethod demo_interface::RobotIOStatus
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:digital_inputs __digital_inputs) (let (r) (dotimes (i 0) (push nil r)) r))
    ((:digital_outputs __digital_outputs) (let (r) (dotimes (i 0) (push nil r)) r))
    ((:analog_inputs __analog_inputs) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:analog_outputs __analog_outputs) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:tool_io_status __tool_io_status) (instance demo_interface::ToolIOStatus :init))
    ((:is_connected __is_connected) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _digital_inputs __digital_inputs)
   (setq _digital_outputs __digital_outputs)
   (setq _analog_inputs __analog_inputs)
   (setq _analog_outputs __analog_outputs)
   (setq _tool_io_status __tool_io_status)
   (setq _is_connected __is_connected)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:digital_inputs
   (&optional __digital_inputs)
   (if __digital_inputs (setq _digital_inputs __digital_inputs)) _digital_inputs)
  (:digital_outputs
   (&optional __digital_outputs)
   (if __digital_outputs (setq _digital_outputs __digital_outputs)) _digital_outputs)
  (:analog_inputs
   (&optional __analog_inputs)
   (if __analog_inputs (setq _analog_inputs __analog_inputs)) _analog_inputs)
  (:analog_outputs
   (&optional __analog_outputs)
   (if __analog_outputs (setq _analog_outputs __analog_outputs)) _analog_outputs)
  (:tool_io_status
   (&rest __tool_io_status)
   (if (keywordp (car __tool_io_status))
       (send* _tool_io_status __tool_io_status)
     (progn
       (if __tool_io_status (setq _tool_io_status (car __tool_io_status)))
       _tool_io_status)))
  (:is_connected
   (&optional (__is_connected :null))
   (if (not (eq __is_connected :null)) (setq _is_connected __is_connected)) _is_connected)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool[] _digital_inputs
    (* 1    (length _digital_inputs)) 4
    ;; bool[] _digital_outputs
    (* 1    (length _digital_outputs)) 4
    ;; float32[] _analog_inputs
    (* 4    (length _analog_inputs)) 4
    ;; float32[] _analog_outputs
    (* 4    (length _analog_outputs)) 4
    ;; demo_interface/ToolIOStatus _tool_io_status
    (send _tool_io_status :serialization-length)
    ;; bool _is_connected
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool[] _digital_inputs
     (write-long (length _digital_inputs) s)
     (dotimes (i (length _digital_inputs))
       (if (elt _digital_inputs i) (write-byte -1 s) (write-byte 0 s))
       )
     ;; bool[] _digital_outputs
     (write-long (length _digital_outputs) s)
     (dotimes (i (length _digital_outputs))
       (if (elt _digital_outputs i) (write-byte -1 s) (write-byte 0 s))
       )
     ;; float32[] _analog_inputs
     (write-long (length _analog_inputs) s)
     (dotimes (i (length _analog_inputs))
       (sys::poke (elt _analog_inputs i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _analog_outputs
     (write-long (length _analog_outputs) s)
     (dotimes (i (length _analog_outputs))
       (sys::poke (elt _analog_outputs i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; demo_interface/ToolIOStatus _tool_io_status
       (send _tool_io_status :serialize s)
     ;; bool _is_connected
       (if _is_connected (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool[] _digital_inputs
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _digital_inputs (make-list n))
     (dotimes (i n)
     (setf (elt _digital_inputs i) (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
     ))
   ;; bool[] _digital_outputs
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _digital_outputs (make-list n))
     (dotimes (i n)
     (setf (elt _digital_outputs i) (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
     ))
   ;; float32[] _analog_inputs
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _analog_inputs (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _analog_inputs i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _analog_outputs
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _analog_outputs (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _analog_outputs i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; demo_interface/ToolIOStatus _tool_io_status
     (send _tool_io_status :deserialize buf ptr-) (incf ptr- (send _tool_io_status :serialization-length))
   ;; bool _is_connected
     (setq _is_connected (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get demo_interface::RobotIOStatus :md5sum-) "241d2a7d7ac2df6c1d3ad9602be6f25a")
(setf (get demo_interface::RobotIOStatus :datatype-) "demo_interface/RobotIOStatus")
(setf (get demo_interface::RobotIOStatus :definition-)
      "std_msgs/Header header
bool[] digital_inputs
bool[] digital_outputs
float32[] analog_inputs
float32[] analog_outputs
ToolIOStatus tool_io_status
bool is_connected


================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: demo_interface/ToolIOStatus
bool[] digital_inputs
bool[] digital_outputs
float32[] analog_inputs
float32[] analog_outputs


")



(provide :demo_interface/RobotIOStatus "241d2a7d7ac2df6c1d3ad9602be6f25a")


