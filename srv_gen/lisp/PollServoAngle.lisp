; Auto-generated. Do not edit!


(cl:in-package module_controller-srv)


;//! \htmlinclude PollServoAngle-request.msg.html

(cl:defclass <PollServoAngle-request> (roslisp-msg-protocol:ros-message)
  ((ID
    :reader ID
    :initarg :ID
    :type cl:fixnum
    :initform 0))
)

(cl:defclass PollServoAngle-request (<PollServoAngle-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PollServoAngle-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PollServoAngle-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name module_controller-srv:<PollServoAngle-request> is deprecated: use module_controller-srv:PollServoAngle-request instead.")))

(cl:ensure-generic-function 'ID-val :lambda-list '(m))
(cl:defmethod ID-val ((m <PollServoAngle-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader module_controller-srv:ID-val is deprecated.  Use module_controller-srv:ID instead.")
  (ID m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PollServoAngle-request>) ostream)
  "Serializes a message object of type '<PollServoAngle-request>"
  (cl:let* ((signed (cl:slot-value msg 'ID)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PollServoAngle-request>) istream)
  "Deserializes a message object of type '<PollServoAngle-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ID) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PollServoAngle-request>)))
  "Returns string type for a service object of type '<PollServoAngle-request>"
  "module_controller/PollServoAngleRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PollServoAngle-request)))
  "Returns string type for a service object of type 'PollServoAngle-request"
  "module_controller/PollServoAngleRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PollServoAngle-request>)))
  "Returns md5sum for a message object of type '<PollServoAngle-request>"
  "58742bd1a1a5aa07bee05a873ed0f9aa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PollServoAngle-request)))
  "Returns md5sum for a message object of type 'PollServoAngle-request"
  "58742bd1a1a5aa07bee05a873ed0f9aa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PollServoAngle-request>)))
  "Returns full string definition for message of type '<PollServoAngle-request>"
  (cl:format cl:nil "int8 ID~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PollServoAngle-request)))
  "Returns full string definition for message of type 'PollServoAngle-request"
  (cl:format cl:nil "int8 ID~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PollServoAngle-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PollServoAngle-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PollServoAngle-request
    (cl:cons ':ID (ID msg))
))
;//! \htmlinclude PollServoAngle-response.msg.html

(cl:defclass <PollServoAngle-response> (roslisp-msg-protocol:ros-message)
  ((angle
    :reader angle
    :initarg :angle
    :type cl:float
    :initform 0.0))
)

(cl:defclass PollServoAngle-response (<PollServoAngle-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PollServoAngle-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PollServoAngle-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name module_controller-srv:<PollServoAngle-response> is deprecated: use module_controller-srv:PollServoAngle-response instead.")))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <PollServoAngle-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader module_controller-srv:angle-val is deprecated.  Use module_controller-srv:angle instead.")
  (angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PollServoAngle-response>) ostream)
  "Serializes a message object of type '<PollServoAngle-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PollServoAngle-response>) istream)
  "Deserializes a message object of type '<PollServoAngle-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PollServoAngle-response>)))
  "Returns string type for a service object of type '<PollServoAngle-response>"
  "module_controller/PollServoAngleResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PollServoAngle-response)))
  "Returns string type for a service object of type 'PollServoAngle-response"
  "module_controller/PollServoAngleResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PollServoAngle-response>)))
  "Returns md5sum for a message object of type '<PollServoAngle-response>"
  "58742bd1a1a5aa07bee05a873ed0f9aa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PollServoAngle-response)))
  "Returns md5sum for a message object of type 'PollServoAngle-response"
  "58742bd1a1a5aa07bee05a873ed0f9aa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PollServoAngle-response>)))
  "Returns full string definition for message of type '<PollServoAngle-response>"
  (cl:format cl:nil "float32 angle~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PollServoAngle-response)))
  "Returns full string definition for message of type 'PollServoAngle-response"
  (cl:format cl:nil "float32 angle~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PollServoAngle-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PollServoAngle-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PollServoAngle-response
    (cl:cons ':angle (angle msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PollServoAngle)))
  'PollServoAngle-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PollServoAngle)))
  'PollServoAngle-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PollServoAngle)))
  "Returns string type for a service object of type '<PollServoAngle>"
  "module_controller/PollServoAngle")
