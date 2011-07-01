; Auto-generated. Do not edit!


(cl:in-package module_controller-srv)


;//! \htmlinclude SetServoSpeed-request.msg.html

(cl:defclass <SetServoSpeed-request> (roslisp-msg-protocol:ros-message)
  ((ID
    :reader ID
    :initarg :ID
    :type cl:fixnum
    :initform 0)
   (Speed
    :reader Speed
    :initarg :Speed
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SetServoSpeed-request (<SetServoSpeed-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetServoSpeed-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetServoSpeed-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name module_controller-srv:<SetServoSpeed-request> is deprecated: use module_controller-srv:SetServoSpeed-request instead.")))

(cl:ensure-generic-function 'ID-val :lambda-list '(m))
(cl:defmethod ID-val ((m <SetServoSpeed-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader module_controller-srv:ID-val is deprecated.  Use module_controller-srv:ID instead.")
  (ID m))

(cl:ensure-generic-function 'Speed-val :lambda-list '(m))
(cl:defmethod Speed-val ((m <SetServoSpeed-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader module_controller-srv:Speed-val is deprecated.  Use module_controller-srv:Speed instead.")
  (Speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetServoSpeed-request>) ostream)
  "Serializes a message object of type '<SetServoSpeed-request>"
  (cl:let* ((signed (cl:slot-value msg 'ID)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'Speed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetServoSpeed-request>) istream)
  "Deserializes a message object of type '<SetServoSpeed-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ID) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Speed) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetServoSpeed-request>)))
  "Returns string type for a service object of type '<SetServoSpeed-request>"
  "module_controller/SetServoSpeedRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetServoSpeed-request)))
  "Returns string type for a service object of type 'SetServoSpeed-request"
  "module_controller/SetServoSpeedRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetServoSpeed-request>)))
  "Returns md5sum for a message object of type '<SetServoSpeed-request>"
  "a5cb71dde39d64eb1c071c1bee620dba")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetServoSpeed-request)))
  "Returns md5sum for a message object of type 'SetServoSpeed-request"
  "a5cb71dde39d64eb1c071c1bee620dba")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetServoSpeed-request>)))
  "Returns full string definition for message of type '<SetServoSpeed-request>"
  (cl:format cl:nil "int8 ID~%int8 Speed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetServoSpeed-request)))
  "Returns full string definition for message of type 'SetServoSpeed-request"
  (cl:format cl:nil "int8 ID~%int8 Speed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetServoSpeed-request>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetServoSpeed-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetServoSpeed-request
    (cl:cons ':ID (ID msg))
    (cl:cons ':Speed (Speed msg))
))
;//! \htmlinclude SetServoSpeed-response.msg.html

(cl:defclass <SetServoSpeed-response> (roslisp-msg-protocol:ros-message)
  ((Success
    :reader Success
    :initarg :Success
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SetServoSpeed-response (<SetServoSpeed-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetServoSpeed-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetServoSpeed-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name module_controller-srv:<SetServoSpeed-response> is deprecated: use module_controller-srv:SetServoSpeed-response instead.")))

(cl:ensure-generic-function 'Success-val :lambda-list '(m))
(cl:defmethod Success-val ((m <SetServoSpeed-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader module_controller-srv:Success-val is deprecated.  Use module_controller-srv:Success instead.")
  (Success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetServoSpeed-response>) ostream)
  "Serializes a message object of type '<SetServoSpeed-response>"
  (cl:let* ((signed (cl:slot-value msg 'Success)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetServoSpeed-response>) istream)
  "Deserializes a message object of type '<SetServoSpeed-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Success) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetServoSpeed-response>)))
  "Returns string type for a service object of type '<SetServoSpeed-response>"
  "module_controller/SetServoSpeedResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetServoSpeed-response)))
  "Returns string type for a service object of type 'SetServoSpeed-response"
  "module_controller/SetServoSpeedResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetServoSpeed-response>)))
  "Returns md5sum for a message object of type '<SetServoSpeed-response>"
  "a5cb71dde39d64eb1c071c1bee620dba")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetServoSpeed-response)))
  "Returns md5sum for a message object of type 'SetServoSpeed-response"
  "a5cb71dde39d64eb1c071c1bee620dba")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetServoSpeed-response>)))
  "Returns full string definition for message of type '<SetServoSpeed-response>"
  (cl:format cl:nil "int8 Success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetServoSpeed-response)))
  "Returns full string definition for message of type 'SetServoSpeed-response"
  (cl:format cl:nil "int8 Success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetServoSpeed-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetServoSpeed-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetServoSpeed-response
    (cl:cons ':Success (Success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetServoSpeed)))
  'SetServoSpeed-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetServoSpeed)))
  'SetServoSpeed-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetServoSpeed)))
  "Returns string type for a service object of type '<SetServoSpeed>"
  "module_controller/SetServoSpeed")
