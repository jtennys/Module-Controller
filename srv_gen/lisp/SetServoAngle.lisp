; Auto-generated. Do not edit!


(cl:in-package module_controller-srv)


;//! \htmlinclude SetServoAngle-request.msg.html

(cl:defclass <SetServoAngle-request> (roslisp-msg-protocol:ros-message)
  ((ID
    :reader ID
    :initarg :ID
    :type cl:fixnum
    :initform 0)
   (Angle
    :reader Angle
    :initarg :Angle
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetServoAngle-request (<SetServoAngle-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetServoAngle-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetServoAngle-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name module_controller-srv:<SetServoAngle-request> is deprecated: use module_controller-srv:SetServoAngle-request instead.")))

(cl:ensure-generic-function 'ID-val :lambda-list '(m))
(cl:defmethod ID-val ((m <SetServoAngle-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader module_controller-srv:ID-val is deprecated.  Use module_controller-srv:ID instead.")
  (ID m))

(cl:ensure-generic-function 'Angle-val :lambda-list '(m))
(cl:defmethod Angle-val ((m <SetServoAngle-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader module_controller-srv:Angle-val is deprecated.  Use module_controller-srv:Angle instead.")
  (Angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetServoAngle-request>) ostream)
  "Serializes a message object of type '<SetServoAngle-request>"
  (cl:let* ((signed (cl:slot-value msg 'ID)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetServoAngle-request>) istream)
  "Deserializes a message object of type '<SetServoAngle-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ID) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Angle) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetServoAngle-request>)))
  "Returns string type for a service object of type '<SetServoAngle-request>"
  "module_controller/SetServoAngleRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetServoAngle-request)))
  "Returns string type for a service object of type 'SetServoAngle-request"
  "module_controller/SetServoAngleRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetServoAngle-request>)))
  "Returns md5sum for a message object of type '<SetServoAngle-request>"
  "c5ba78eaaf56070109b47d88ff7a70ae")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetServoAngle-request)))
  "Returns md5sum for a message object of type 'SetServoAngle-request"
  "c5ba78eaaf56070109b47d88ff7a70ae")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetServoAngle-request>)))
  "Returns full string definition for message of type '<SetServoAngle-request>"
  (cl:format cl:nil "int8 ID~%float32 Angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetServoAngle-request)))
  "Returns full string definition for message of type 'SetServoAngle-request"
  (cl:format cl:nil "int8 ID~%float32 Angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetServoAngle-request>))
  (cl:+ 0
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetServoAngle-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetServoAngle-request
    (cl:cons ':ID (ID msg))
    (cl:cons ':Angle (Angle msg))
))
;//! \htmlinclude SetServoAngle-response.msg.html

(cl:defclass <SetServoAngle-response> (roslisp-msg-protocol:ros-message)
  ((Success
    :reader Success
    :initarg :Success
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SetServoAngle-response (<SetServoAngle-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetServoAngle-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetServoAngle-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name module_controller-srv:<SetServoAngle-response> is deprecated: use module_controller-srv:SetServoAngle-response instead.")))

(cl:ensure-generic-function 'Success-val :lambda-list '(m))
(cl:defmethod Success-val ((m <SetServoAngle-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader module_controller-srv:Success-val is deprecated.  Use module_controller-srv:Success instead.")
  (Success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetServoAngle-response>) ostream)
  "Serializes a message object of type '<SetServoAngle-response>"
  (cl:let* ((signed (cl:slot-value msg 'Success)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetServoAngle-response>) istream)
  "Deserializes a message object of type '<SetServoAngle-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Success) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetServoAngle-response>)))
  "Returns string type for a service object of type '<SetServoAngle-response>"
  "module_controller/SetServoAngleResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetServoAngle-response)))
  "Returns string type for a service object of type 'SetServoAngle-response"
  "module_controller/SetServoAngleResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetServoAngle-response>)))
  "Returns md5sum for a message object of type '<SetServoAngle-response>"
  "c5ba78eaaf56070109b47d88ff7a70ae")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetServoAngle-response)))
  "Returns md5sum for a message object of type 'SetServoAngle-response"
  "c5ba78eaaf56070109b47d88ff7a70ae")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetServoAngle-response>)))
  "Returns full string definition for message of type '<SetServoAngle-response>"
  (cl:format cl:nil "int8 Success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetServoAngle-response)))
  "Returns full string definition for message of type 'SetServoAngle-response"
  (cl:format cl:nil "int8 Success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetServoAngle-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetServoAngle-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetServoAngle-response
    (cl:cons ':Success (Success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetServoAngle)))
  'SetServoAngle-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetServoAngle)))
  'SetServoAngle-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetServoAngle)))
  "Returns string type for a service object of type '<SetServoAngle>"
  "module_controller/SetServoAngle")
