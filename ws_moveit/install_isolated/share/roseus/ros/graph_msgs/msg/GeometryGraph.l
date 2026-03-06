;; Auto-generated. Do not edit!


(when (boundp 'graph_msgs::GeometryGraph)
  (if (not (find-package "GRAPH_MSGS"))
    (make-package "GRAPH_MSGS"))
  (shadow 'GeometryGraph (find-package "GRAPH_MSGS")))
(unless (find-package "GRAPH_MSGS::GEOMETRYGRAPH")
  (make-package "GRAPH_MSGS::GEOMETRYGRAPH"))

(in-package "ROS")
;;//! \htmlinclude GeometryGraph.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass graph_msgs::GeometryGraph
  :super ros::object
  :slots (_header _nodes _edges ))

(defmethod graph_msgs::GeometryGraph
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:nodes __nodes) ())
    ((:edges __edges) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _nodes __nodes)
   (setq _edges __edges)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:nodes
   (&rest __nodes)
   (if (keywordp (car __nodes))
       (send* _nodes __nodes)
     (progn
       (if __nodes (setq _nodes (car __nodes)))
       _nodes)))
  (:edges
   (&rest __edges)
   (if (keywordp (car __edges))
       (send* _edges __edges)
     (progn
       (if __edges (setq _edges (car __edges)))
       _edges)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Point[] _nodes
    (apply #'+ (send-all _nodes :serialization-length)) 4
    ;; graph_msgs/Edges[] _edges
    (apply #'+ (send-all _edges :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Point[] _nodes
     (write-long (length _nodes) s)
     (dolist (elem _nodes)
       (send elem :serialize s)
       )
     ;; graph_msgs/Edges[] _edges
     (write-long (length _edges) s)
     (dolist (elem _edges)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Point[] _nodes
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _nodes (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _nodes)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; graph_msgs/Edges[] _edges
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _edges (let (r) (dotimes (i n) (push (instance graph_msgs::Edges :init) r)) r))
     (dolist (elem- _edges)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get graph_msgs::GeometryGraph :md5sum-) "78739617daca94d38915923795eada2d")
(setf (get graph_msgs::GeometryGraph :datatype-) "graph_msgs/GeometryGraph")
(setf (get graph_msgs::GeometryGraph :definition-)
      "# A reference coordinate frame and timestamp
Header header

# 3D spacial graph
geometry_msgs/Point[] nodes

# This vector should be the same length as nodes, above, and represents all the connecting nodes
Edges[] edges

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
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: graph_msgs/Edges
#base-zero index of all the verticies this particular vertice connects to (edges)
uint32[] node_ids

# optional cost/weight of each edge. if vector is empty assume all weights are equal (1)
float64[] weights
")



(provide :graph_msgs/GeometryGraph "78739617daca94d38915923795eada2d")


