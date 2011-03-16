; Auto-generated. Do not edit!


(cl:in-package module_controller-srv)


;//! \htmlinclude GetServoAngle-request.msg.html

(cl:defclass <GetServoAngle-request> (roslisp-msg-protocol:ros-message)
  ((ID
    :reader ID
    :initarg :ID
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GetServoAngle-request (<GetServoAngle-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetServoAngle-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetServoAngle-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name module_controller-srv:<GetServoAngle-request> is deprecated: use module_controller-srv:GetServoAngle-request instead.")))

(cl:ensure-generic-function 'ID-val :lambda-list '(m))
(cl:defmethod ID-val ((m <GetServoAngle-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader module_controller-srv:ID-val is deprecated.  Use module_controller-srv:ID instead.")
  (ID m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetServoAngle-request>) ostream)
  "Serializes a message object of type '<GetServoAngle-request>"
  (cl:let* ((signed (cl:slot-value msg 'ID)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetServoAngle-request>) istream)
  "Deserializes a message object of type '<GetServoAngle-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ID) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetServoAngle-request>)))
  "Returns string type for a service object of type '<GetServoAngle-request>"
  "module_controller/GetServoAngleRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetServoAngle-request)))
  "Returns string type for a service object of type 'GetServoAngle-request"
  "module_controller/GetServoAngleRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetServoAngle-request>)))
  "Returns md5sum for a message object of type '<GetServoAngle-request>"
  "3b3d703516b54eab670621d84fbecd83")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetServoAngle-request)))
  "Returns md5sum for a message object of type 'GetServoAngle-request"
  "3b3d703516b54eab670621d84fbecd83")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetServoAngle-request>)))
  "Returns full string definition for message of type '<GetServoAngle-request>"
  (cl:format cl:nil "int8 ID~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetServoAngle-request)))
  "Returns full string definition for message of type 'GetServoAngle-request"
  (cl:format cl:nil "int8 ID~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetServoAngle-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetServoAngle-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetServoAngle-request
    (cl:cons ':ID (ID msg))
))
;//! \htmlinclude GetServoAngle-response.msg.html

(cl:defclass <GetServoAngle-response> (roslisp-msg-protocol:ros-message)
  ((Angle
    :reader Angle
    :initarg :Angle
    :type cl:float
    :initform 0.0))
)

(cl:defclass GetServoAngle-response (<GetServoAngle-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetServoAngle-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetServoAngle-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name module_controller-srv:<GetServoAngle-response> is deprecated: use module_controller-srv:GetServoAngle-response instead.")))

(cl:ensure-generic-function 'Angle-val :lambda-list '(m))
(cl:defmethod Angle-val ((m <GetServoAngle-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader module_controller-srv:Angle-val is deprecated.  Use module_controller-srv:Angle instead.")
  (Angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetServoAngle-response>) ostream)
  "Serializes a message object of type '<GetServoAngle-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetServoAngle-response>) istream)
  "Deserializes a message object of type '<GetServoAngle-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Angle) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetServoAngle-response>)))
  "Returns string type for a service object of type '<GetServoAngle-response>"
  "module_controller/GetServoAngleResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetServoAngle-response)))
  "Returns string type for a service object of type 'GetServoAngle-response"
  "module_controller/GetServoAngleResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetServoAngle-response>)))
  "Returns md5sum for a message object of type '<GetServoAngle-response>"
  "3b3d703516b54eab670621d84fbecd83")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetServoAngle-response)))
  "Returns md5sum for a message object of type 'GetServoAngle-response"
  "3b3d703516b54eab670621d84fbecd83")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetServoAngle-response>)))
  "Returns full string definition for message of type '<GetServoAngle-response>"
  (cl:format cl:nil "float32 Angle~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetServoAngle-response)))
  "Returns full string definition for message of type 'GetServoAngle-response"
  (cl:format cl:nil "float32 Angle~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetServoAngle-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetServoAngle-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetServoAngle-response
    (cl:cons ':Angle (Angle msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetServoAngle)))
  'GetServoAngle-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetServoAngle)))
  'GetServoAngle-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetServoAngle)))
  "Returns string type for a service object of type '<GetServoAngle>"
  "module_controller/GetServoAngle")
