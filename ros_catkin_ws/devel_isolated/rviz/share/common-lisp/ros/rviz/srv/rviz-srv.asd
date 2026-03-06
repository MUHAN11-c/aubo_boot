
(cl:in-package :asdf)

(defsystem "rviz-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "SendFilePath" :depends-on ("_package_SendFilePath"))
    (:file "_package_SendFilePath" :depends-on ("_package"))
  ))