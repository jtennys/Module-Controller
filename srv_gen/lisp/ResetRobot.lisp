; Auto-generated. Do not edit!


(cl:in-package module_controller-srv)


;//! \htmlinclude ResetRobot-request.msg.html

(cl:defclass <ResetRobot-request> (roslisp-msg-protocol:ros-message)
  ((Nothing
    :reader Nothing
    :initarg :Nothing
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ResetRobot-request (<ResetRobot-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ResetRobot-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ResetRobot-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name module_controller-srv:<ResetRobot-request> is deprecated: use module_controller-srv:ResetRobot-request instead.")))

(cl:ensure-generic-function 'Nothing-val :lambda-list '(m))
(cl:defmethod Nothing-val ((m <ResetRobot-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader module_controller-srv:Nothing-val is deprecated.  Use module_controller-srv:Nothing instead.")
  (Nothing m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ResetRobot-request>) ostream)
  "Serializes a message object of type '<ResetRobot-request>"
  (cl:let* ((signed (cl:slot-value msg 'Nothing)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ResetRobot-request>) istream)
  "Deserializes a message object of type '<ResetRobot-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Nothing) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ResetRobot-request>)))
  "Returns string type for a service object of type '<ResetRobot-request>"
  "module_controller/ResetRobotRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetRobot-request)))
  "Returns string type for a service object of type 'ResetRobot-request"
  "module_controller/ResetRobotRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ResetRobot-request>)))
  "Returns md5sum for a message object of type '<ResetRobot-request>"
  "4a1954151a0c2d771839ed336f9b5647")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ResetRobot-request)))
  "Returns md5sum for a message object of type 'ResetRobot-request"
  "4a1954151a0c2d771839ed336f9b5647")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ResetRobot-request>)))
  "Returns full string definition for message of type '<ResetRobot-request>"
  (cl:format cl:nil "int8 Nothing~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ResetRobot-request)))
  "Returns full string definition for message of type 'ResetRobot-request"
  (cl:format cl:nil "int8 Nothing~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ResetRobot-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ResetRobot-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ResetRobot-request
    (cl:cons ':Nothing (Nothing msg))
))
;//! \htmlinclude ResetRobot-response.msg.html

(cl:defclass <ResetRobot-response> (roslisp-msg-protocol:ros-message)
  ((Success
    :reader Success
    :initarg :Success
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ResetRobot-response (<ResetRobot-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ResetRobot-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ResetRobot-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name module_controller-srv:<ResetRobot-response> is deprecated: use module_controller-srv:ResetRobot-response instead.")))

(cl:ensure-generic-function 'Success-val :lambda-list '(m))
(cl:defmethod Success-val ((m <ResetRobot-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader module_controller-srv:Success-val is deprecated.  Use module_controller-srv:Success instead.")
  (Success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ResetRobot-response>) ostream)
  "Serializes a message object of type '<ResetRobot-response>"
  (cl:let* ((signed (cl:slot-value msg 'Success)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ResetRobot-response>) istream)
  "Deserializes a message object of type '<ResetRobot-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Success) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ResetRobot-response>)))
  "Returns string type for a service object of type '<ResetRobot-response>"
  "module_controller/ResetRobotResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetRobot-response)))
  "Returns string type for a service object of type 'ResetRobot-response"
  "module_controller/ResetRobotResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ResetRobot-response>)))
  "Returns md5sum for a message object of type '<ResetRobot-response>"
  "4a1954151a0c2d771839ed336f9b5647")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ResetRobot-response)))
  "Returns md5sum for a message object of type 'ResetRobot-response"
  "4a1954151a0c2d771839ed336f9b5647")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ResetRobot-response>)))
  "Returns full string definition for message of type '<ResetRobot-response>"
  (cl:format cl:nil "int8 Success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ResetRobot-response)))
  "Returns full string definition for message of type 'ResetRobot-response"
  (cl:format cl:nil "int8 Success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ResetRobot-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ResetRobot-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ResetRobot-response
    (cl:cons ':Success (Success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ResetRobot)))
  'ResetRobot-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ResetRobot)))
  'ResetRobot-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetRobot)))
  "Returns string type for a service object of type '<ResetRobot>"
  "module_controller/ResetRobot")
