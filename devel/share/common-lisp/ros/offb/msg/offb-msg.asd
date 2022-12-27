
(cl:in-package :asdf)

(defsystem "offb-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "obj" :depends-on ("_package_obj"))
    (:file "_package_obj" :depends-on ("_package"))
  ))