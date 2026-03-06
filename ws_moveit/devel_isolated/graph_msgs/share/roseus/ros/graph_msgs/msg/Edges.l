;; Auto-generated. Do not edit!


(when (boundp 'graph_msgs::Edges)
  (if (not (find-package "GRAPH_MSGS"))
    (make-package "GRAPH_MSGS"))
  (shadow 'Edges (find-package "GRAPH_MSGS")))
(unless (find-package "GRAPH_MSGS::EDGES")
  (make-package "GRAPH_MSGS::EDGES"))

(in-package "ROS")
;;//! \htmlinclude Edges.msg.html


(defclass graph_msgs::Edges
  :super ros::object
  :slots (_node_ids _weights ))

(defmethod graph_msgs::Edges
  (:init
   (&key
    ((:node_ids __node_ids) (make-array 0 :initial-element 0 :element-type :integer))
    ((:weights __weights) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _node_ids __node_ids)
   (setq _weights __weights)
   self)
  (:node_ids
   (&optional __node_ids)
   (if __node_ids (setq _node_ids __node_ids)) _node_ids)
  (:weights
   (&optional __weights)
   (if __weights (setq _weights __weights)) _weights)
  (:serialization-length
   ()
   (+
    ;; uint32[] _node_ids
    (* 4    (length _node_ids)) 4
    ;; float64[] _weights
    (* 8    (length _weights)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32[] _node_ids
     (write-long (length _node_ids) s)
     (dotimes (i (length _node_ids))
       (write-long (elt _node_ids i) s)
       )
     ;; float64[] _weights
     (write-long (length _weights) s)
     (dotimes (i (length _weights))
       (sys::poke (elt _weights i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32[] _node_ids
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _node_ids (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _node_ids i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; float64[] _weights
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _weights (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _weights i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get graph_msgs::Edges :md5sum-) "1dcd54afd0b0c0fbebeb59dbdda4c026")
(setf (get graph_msgs::Edges :datatype-) "graph_msgs/Edges")
(setf (get graph_msgs::Edges :definition-)
      "#base-zero index of all the verticies this particular vertice connects to (edges)
uint32[] node_ids

# optional cost/weight of each edge. if vector is empty assume all weights are equal (1)
float64[] weights
")



(provide :graph_msgs/Edges "1dcd54afd0b0c0fbebeb59dbdda4c026")


