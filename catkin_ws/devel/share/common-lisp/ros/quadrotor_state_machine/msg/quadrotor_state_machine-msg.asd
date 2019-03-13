
(cl:in-package :asdf)

(defsystem "quadrotor_state_machine-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "StateCommand" :depends-on ("_package_StateCommand"))
    (:file "_package_StateCommand" :depends-on ("_package"))
  ))