
(cl:in-package :asdf)

(defsystem "hippo2-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Int" :depends-on ("_package_Int"))
    (:file "_package_Int" :depends-on ("_package"))
  ))