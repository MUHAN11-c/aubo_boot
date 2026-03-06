;; Auto-generated. Do not edit!


(when (boundp 'object_recognition_msgs::ObjectType)
  (if (not (find-package "OBJECT_RECOGNITION_MSGS"))
    (make-package "OBJECT_RECOGNITION_MSGS"))
  (shadow 'ObjectType (find-package "OBJECT_RECOGNITION_MSGS")))
(unless (find-package "OBJECT_RECOGNITION_MSGS::OBJECTTYPE")
  (make-package "OBJECT_RECOGNITION_MSGS::OBJECTTYPE"))

(in-package "ROS")
;;//! \htmlinclude ObjectType.msg.html


(defclass object_recognition_msgs::ObjectType
  :super ros::object
  :slots (_key _db ))

(defmethod object_recognition_msgs::ObjectType
  (:init
   (&key
    ((:key __key) "")
    ((:db __db) "")
    )
   (send-super :init)
   (setq _key (string __key))
   (setq _db (string __db))
   self)
  (:key
   (&optional __key)
   (if __key (setq _key __key)) _key)
  (:db
   (&optional __db)
   (if __db (setq _db __db)) _db)
  (:serialization-length
   ()
   (+
    ;; string _key
    4 (length _key)
    ;; string _db
    4 (length _db)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _key
       (write-long (length _key) s) (princ _key s)
     ;; string _db
       (write-long (length _db) s) (princ _db s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _key
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _key (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _db
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _db (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get object_recognition_msgs::ObjectType :md5sum-) "ac757ec5be1998b0167e7efcda79e3cf")
(setf (get object_recognition_msgs::ObjectType :datatype-) "object_recognition_msgs/ObjectType")
(setf (get object_recognition_msgs::ObjectType :definition-)
      "################################################## OBJECT ID #########################################################

# Contains information about the type of a found object. Those two sets of parameters together uniquely define an
# object

# The key of the found object: the unique identifier in the given db
string key

# The db parameters stored as a JSON/compressed YAML string. An object id does not make sense without the corresponding
# database. E.g., in object_recognition, it can look like: \"{'type':'CouchDB', 'root':'http://localhost'}\"
# There is no conventional format for those parameters and it's nice to keep that flexibility.
# The object_recognition_core as a generic DB type that can read those fields
# Current examples:
# For CouchDB:
#   type: 'CouchDB'
#   root: 'http://localhost:5984'
#   collection: 'object_recognition'
# For SQL household database:
#   type: 'SqlHousehold'
#   host: 'wgs36'
#   port: 5432
#   user: 'willow'
#   password: 'willow'
#   name: 'household_objects'
#   module: 'tabletop'
string db

")



(provide :object_recognition_msgs/ObjectType "ac757ec5be1998b0167e7efcda79e3cf")


