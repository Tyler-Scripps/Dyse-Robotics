
(cl:in-package :asdf)

(defsystem "arman_controller-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ArmanDriverRequest" :depends-on ("_package_ArmanDriverRequest"))
    (:file "_package_ArmanDriverRequest" :depends-on ("_package"))
  ))