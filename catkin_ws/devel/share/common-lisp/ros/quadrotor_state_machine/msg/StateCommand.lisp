; Auto-generated. Do not edit!


(cl:in-package quadrotor_state_machine-msg)


;//! \htmlinclude StateCommand.msg.html

(cl:defclass <StateCommand> (roslisp-msg-protocol:ros-message)
  ((next
    :reader next
    :initarg :next
    :type cl:string
    :initform "")
   (current
    :reader current
    :initarg :current
    :type cl:string
    :initform ""))
)

(cl:defclass StateCommand (<StateCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StateCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StateCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name quadrotor_state_machine-msg:<StateCommand> is deprecated: use quadrotor_state_machine-msg:StateCommand instead.")))

(cl:ensure-generic-function 'next-val :lambda-list '(m))
(cl:defmethod next-val ((m <StateCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_state_machine-msg:next-val is deprecated.  Use quadrotor_state_machine-msg:next instead.")
  (next m))

(cl:ensure-generic-function 'current-val :lambda-list '(m))
(cl:defmethod current-val ((m <StateCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader quadrotor_state_machine-msg:current-val is deprecated.  Use quadrotor_state_machine-msg:current instead.")
  (current m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StateCommand>) ostream)
  "Serializes a message object of type '<StateCommand>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'next))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'next))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'current))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'current))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StateCommand>) istream)
  "Deserializes a message object of type '<StateCommand>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'next) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'next) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'current) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'current) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StateCommand>)))
  "Returns string type for a message object of type '<StateCommand>"
  "quadrotor_state_machine/StateCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StateCommand)))
  "Returns string type for a message object of type 'StateCommand"
  "quadrotor_state_machine/StateCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StateCommand>)))
  "Returns md5sum for a message object of type '<StateCommand>"
  "928bf34b732b7e32d1524eb9acbd1715")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StateCommand)))
  "Returns md5sum for a message object of type 'StateCommand"
  "928bf34b732b7e32d1524eb9acbd1715")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StateCommand>)))
  "Returns full string definition for message of type '<StateCommand>"
  (cl:format cl:nil "string next~%string current~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StateCommand)))
  "Returns full string definition for message of type 'StateCommand"
  (cl:format cl:nil "string next~%string current~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StateCommand>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'next))
     4 (cl:length (cl:slot-value msg 'current))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StateCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'StateCommand
    (cl:cons ':next (next msg))
    (cl:cons ':current (current msg))
))
