
(cl:in-package :asdf)

(defsystem "object_recognition_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :object_recognition_msgs-msg
)
  :components ((:file "_package")
    (:file "GetObjectInformation" :depends-on ("_package_GetObjectInformation"))
    (:file "_package_GetObjectInformation" :depends-on ("_package"))
  ))