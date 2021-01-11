; Auto-generated. Do not edit!


(cl:in-package arman_controller-srv)


;//! \htmlinclude ArmanDriverRequest-request.msg.html

(cl:defclass <ArmanDriverRequest-request> (roslisp-msg-protocol:ros-message)
  ((joint0
    :reader joint0
    :initarg :joint0
    :type cl:float
    :initform 0.0)
   (joint1
    :reader joint1
    :initarg :joint1
    :type cl:float
    :initform 0.0)
   (joint2
    :reader joint2
    :initarg :joint2
    :type cl:float
    :initform 0.0)
   (joint3
    :reader joint3
    :initarg :joint3
    :type cl:float
    :initform 0.0)
   (joint4
    :reader joint4
    :initarg :joint4
    :type cl:float
    :initform 0.0))
)

(cl:defclass ArmanDriverRequest-request (<ArmanDriverRequest-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ArmanDriverRequest-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ArmanDriverRequest-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name arman_controller-srv:<ArmanDriverRequest-request> is deprecated: use arman_controller-srv:ArmanDriverRequest-request instead.")))

(cl:ensure-generic-function 'joint0-val :lambda-list '(m))
(cl:defmethod joint0-val ((m <ArmanDriverRequest-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arman_controller-srv:joint0-val is deprecated.  Use arman_controller-srv:joint0 instead.")
  (joint0 m))

(cl:ensure-generic-function 'joint1-val :lambda-list '(m))
(cl:defmethod joint1-val ((m <ArmanDriverRequest-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arman_controller-srv:joint1-val is deprecated.  Use arman_controller-srv:joint1 instead.")
  (joint1 m))

(cl:ensure-generic-function 'joint2-val :lambda-list '(m))
(cl:defmethod joint2-val ((m <ArmanDriverRequest-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arman_controller-srv:joint2-val is deprecated.  Use arman_controller-srv:joint2 instead.")
  (joint2 m))

(cl:ensure-generic-function 'joint3-val :lambda-list '(m))
(cl:defmethod joint3-val ((m <ArmanDriverRequest-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arman_controller-srv:joint3-val is deprecated.  Use arman_controller-srv:joint3 instead.")
  (joint3 m))

(cl:ensure-generic-function 'joint4-val :lambda-list '(m))
(cl:defmethod joint4-val ((m <ArmanDriverRequest-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arman_controller-srv:joint4-val is deprecated.  Use arman_controller-srv:joint4 instead.")
  (joint4 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ArmanDriverRequest-request>) ostream)
  "Serializes a message object of type '<ArmanDriverRequest-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'joint0))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'joint1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'joint2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'joint3))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'joint4))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ArmanDriverRequest-request>) istream)
  "Deserializes a message object of type '<ArmanDriverRequest-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'joint0) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'joint1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'joint2) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'joint3) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'joint4) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ArmanDriverRequest-request>)))
  "Returns string type for a service object of type '<ArmanDriverRequest-request>"
  "arman_controller/ArmanDriverRequestRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ArmanDriverRequest-request)))
  "Returns string type for a service object of type 'ArmanDriverRequest-request"
  "arman_controller/ArmanDriverRequestRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ArmanDriverRequest-request>)))
  "Returns md5sum for a message object of type '<ArmanDriverRequest-request>"
  "32056f59691abbe99bbd41d6989295c4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ArmanDriverRequest-request)))
  "Returns md5sum for a message object of type 'ArmanDriverRequest-request"
  "32056f59691abbe99bbd41d6989295c4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ArmanDriverRequest-request>)))
  "Returns full string definition for message of type '<ArmanDriverRequest-request>"
  (cl:format cl:nil "float32 joint0~%float32 joint1~%float32 joint2~%float32 joint3~%float32 joint4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ArmanDriverRequest-request)))
  "Returns full string definition for message of type 'ArmanDriverRequest-request"
  (cl:format cl:nil "float32 joint0~%float32 joint1~%float32 joint2~%float32 joint3~%float32 joint4~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ArmanDriverRequest-request>))
  (cl:+ 0
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ArmanDriverRequest-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ArmanDriverRequest-request
    (cl:cons ':joint0 (joint0 msg))
    (cl:cons ':joint1 (joint1 msg))
    (cl:cons ':joint2 (joint2 msg))
    (cl:cons ':joint3 (joint3 msg))
    (cl:cons ':joint4 (joint4 msg))
))
;//! \htmlinclude ArmanDriverRequest-response.msg.html

(cl:defclass <ArmanDriverRequest-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ArmanDriverRequest-response (<ArmanDriverRequest-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ArmanDriverRequest-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ArmanDriverRequest-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name arman_controller-srv:<ArmanDriverRequest-response> is deprecated: use arman_controller-srv:ArmanDriverRequest-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <ArmanDriverRequest-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arman_controller-srv:status-val is deprecated.  Use arman_controller-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ArmanDriverRequest-response>) ostream)
  "Serializes a message object of type '<ArmanDriverRequest-response>"
  (cl:let* ((signed (cl:slot-value msg 'status)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ArmanDriverRequest-response>) istream)
  "Deserializes a message object of type '<ArmanDriverRequest-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ArmanDriverRequest-response>)))
  "Returns string type for a service object of type '<ArmanDriverRequest-response>"
  "arman_controller/ArmanDriverRequestResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ArmanDriverRequest-response)))
  "Returns string type for a service object of type 'ArmanDriverRequest-response"
  "arman_controller/ArmanDriverRequestResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ArmanDriverRequest-response>)))
  "Returns md5sum for a message object of type '<ArmanDriverRequest-response>"
  "32056f59691abbe99bbd41d6989295c4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ArmanDriverRequest-response)))
  "Returns md5sum for a message object of type 'ArmanDriverRequest-response"
  "32056f59691abbe99bbd41d6989295c4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ArmanDriverRequest-response>)))
  "Returns full string definition for message of type '<ArmanDriverRequest-response>"
  (cl:format cl:nil "int8 status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ArmanDriverRequest-response)))
  "Returns full string definition for message of type 'ArmanDriverRequest-response"
  (cl:format cl:nil "int8 status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ArmanDriverRequest-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ArmanDriverRequest-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ArmanDriverRequest-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ArmanDriverRequest)))
  'ArmanDriverRequest-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ArmanDriverRequest)))
  'ArmanDriverRequest-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ArmanDriverRequest)))
  "Returns string type for a service object of type '<ArmanDriverRequest>"
  "arman_controller/ArmanDriverRequest")