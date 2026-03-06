
(cl:in-package :asdf)

(defsystem "graph_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Edges" :depends-on ("_package_Edges"))
    (:file "_package_Edges" :depends-on ("_package"))
    (:file "GeometryGraph" :depends-on ("_package_GeometryGraph"))
    (:file "_package_GeometryGraph" :depends-on ("_package"))
  ))