;; Auto-generated. Do not edit!


(when (boundp 'quadrotor_state_machine::StateCommand)
  (if (not (find-package "QUADROTOR_STATE_MACHINE"))
    (make-package "QUADROTOR_STATE_MACHINE"))
  (shadow 'StateCommand (find-package "QUADROTOR_STATE_MACHINE")))
(unless (find-package "QUADROTOR_STATE_MACHINE::STATECOMMAND")
  (make-package "QUADROTOR_STATE_MACHINE::STATECOMMAND"))

(in-package "ROS")
;;//! \htmlinclude StateCommand.msg.html


(defclass quadrotor_state_machine::StateCommand
  :super ros::object
  :slots (_next _current ))

(defmethod quadrotor_state_machine::StateCommand
  (:init
   (&key
    ((:next __next) "")
    ((:current __current) "")
    )
   (send-super :init)
   (setq _next (string __next))
   (setq _current (string __current))
   self)
  (:next
   (&optional __next)
   (if __next (setq _next __next)) _next)
  (:current
   (&optional __current)
   (if __current (setq _current __current)) _current)
  (:serialization-length
   ()
   (+
    ;; string _next
    4 (length _next)
    ;; string _current
    4 (length _current)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _next
       (write-long (length _next) s) (princ _next s)
     ;; string _current
       (write-long (length _current) s) (princ _current s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _next
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _next (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _current
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _current (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get quadrotor_state_machine::StateCommand :md5sum-) "928bf34b732b7e32d1524eb9acbd1715")
(setf (get quadrotor_state_machine::StateCommand :datatype-) "quadrotor_state_machine/StateCommand")
(setf (get quadrotor_state_machine::StateCommand :definition-)
      "string next
string current

")



(provide :quadrotor_state_machine/StateCommand "928bf34b732b7e32d1524eb9acbd1715")


