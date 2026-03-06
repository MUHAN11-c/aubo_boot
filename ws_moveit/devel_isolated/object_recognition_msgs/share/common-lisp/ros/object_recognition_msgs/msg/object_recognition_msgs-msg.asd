
(cl:in-package :asdf)

(defsystem "object_recognition_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :actionlib_msgs-msg
               :geometry_msgs-msg
               :sensor_msgs-msg
               :shape_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ObjectInformation" :depends-on ("_package_ObjectInformation"))
    (:file "_package_ObjectInformation" :depends-on ("_package"))
    (:file "ObjectRecognitionAction" :depends-on ("_package_ObjectRecognitionAction"))
    (:file "_package_ObjectRecognitionAction" :depends-on ("_package"))
    (:file "ObjectRecognitionActionFeedback" :depends-on ("_package_ObjectRecognitionActionFeedback"))
    (:file "_package_ObjectRecognitionActionFeedback" :depends-on ("_package"))
    (:file "ObjectRecognitionActionGoal" :depends-on ("_package_ObjectRecognitionActionGoal"))
    (:file "_package_ObjectRecognitionActionGoal" :depends-on ("_package"))
    (:file "ObjectRecognitionActionResult" :depends-on ("_package_ObjectRecognitionActionResult"))
    (:file "_package_ObjectRecognitionActionResult" :depends-on ("_package"))
    (:file "ObjectRecognitionFeedback" :depends-on ("_package_ObjectRecognitionFeedback"))
    (:file "_package_ObjectRecognitionFeedback" :depends-on ("_package"))
    (:file "ObjectRecognitionGoal" :depends-on ("_package_ObjectRecognitionGoal"))
    (:file "_package_ObjectRecognitionGoal" :depends-on ("_package"))
    (:file "ObjectRecognitionResult" :depends-on ("_package_ObjectRecognitionResult"))
    (:file "_package_ObjectRecognitionResult" :depends-on ("_package"))
    (:file "ObjectType" :depends-on ("_package_ObjectType"))
    (:file "_package_ObjectType" :depends-on ("_package"))
    (:file "RecognizedObject" :depends-on ("_package_RecognizedObject"))
    (:file "_package_RecognizedObject" :depends-on ("_package"))
    (:file "RecognizedObjectArray" :depends-on ("_package_RecognizedObjectArray"))
    (:file "_package_RecognizedObjectArray" :depends-on ("_package"))
    (:file "Table" :depends-on ("_package_Table"))
    (:file "_package_Table" :depends-on ("_package"))
    (:file "TableArray" :depends-on ("_package_TableArray"))
    (:file "_package_TableArray" :depends-on ("_package"))
  ))