; Auto-generated. Do not edit!


(cl:in-package module_controller-srv)


;//! \htmlinclude GetServoPower-request.msg.html

(cl:defclass <GetServoPower-request> (roslisp-msg-protocol:ros-message)
  ((ID
    :reader ID
    :initarg :ID
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GetServoPower-request (<GetServoPower-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetServoPower-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetServoPower-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name module_controller-srv:<GetServoPower-request> is deprecated: use module_controller-srv:GetServoPower-request instead.")))

(cl:ensure-generic-function 'ID-val :lambda-list '(m))
(cl:defmethod ID-val ((m <GetServoPower-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader module_controller-srv:ID-val is deprecated.  Use module_controller-srv:ID instead.")
  (ID m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetServoPower-request>) ostream)
  "Serializes a message object of type '<GetServoPower-request>"
  (cl:let* ((signed (cl:slot-value msg 'ID)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetServoPower-request>) istream)
  "Deserializes a message object of type '<GetServoPower-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ID) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetServoPower-request>)))
  "Returns string type for a service object of type '<GetServoPower-request>"
  "module_controller/GetServoPowerRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetServoPower-request)))
  "Returns string type for a service object of type 'GetServoPower-request"
  "module_controller/GetServoPowerRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetServoPower-request>)))
  "Returns md5sum for a message object of type '<GetServoPower-request>"
  "63625bde14a8229e56f4eaf5667c7359")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetServoPower-request)))
  "Returns md5sum for a message object of type 'GetServoPower-request"
  "63625bde14a8229e56f4eaf5667c7359")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetServoPower-request>)))
  "Returns full string definition for message of type '<GetServoPower-request>"
  (cl:format cl:nil "int8 ID~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetServoPower-request)))
  "Returns full string definition for message of type 'GetServoPower-request"
  (cl:format cl:nil "int8 ID~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetServoPower-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetServoPower-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetServoPower-request
    (cl:cons ':ID (ID msg))
))
;//! \htmlinclude GetServoPower-response.msg.html

(cl:defclass <GetServoPower-response> (roslisp-msg-protocol:ros-message)
  ((Power
    :reader Power
    :initarg :Power
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GetServoPower-response (<GetServoPower-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetServoPower-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetServoPower-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name module_controller-srv:<GetServoPower-response> is deprecated: use module_controller-srv:GetServoPower-response instead.")))

(cl:ensure-generic-function 'Power-val :lambda-list '(m))
(cl:defmethod Power-val ((m <GetServoPower-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader module_controller-srv:Power-val is deprecated.  Use module_controller-srv:Power instead.")
  (Power m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetServoPower-response>) ostream)
  "Serializes a message object of type '<GetServoPower-response>"
  (cl:let* ((signed (cl:slot-value msg 'Power)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetServoPower-response>) istream)
  "Deserializes a message object of type '<GetServoPower-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Power) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetServoPower-response>)))
  "Returns string type for a service object of type '<GetServoPower-response>"
  "module_controller/GetServoPowerResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetServoPower-response)))
  "Returns string type for a service object of type 'GetServoPower-response"
  "module_controller/GetServoPowerResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetServoPower-response>)))
  "Returns md5sum for a message object of type '<GetServoPower-response>"
  "63625bde14a8229e56f4eaf5667c7359")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetServoPower-response)))
  "Returns md5sum for a message object of type 'GetServoPower-response"
  "63625bde14a8229e56f4eaf5667c7359")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetServoPower-response>)))
  "Returns full string definition for message of type '<GetServoPower-response>"
  (cl:format cl:nil "int8 Power~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetServoPower-response)))
  "Returns full string definition for message of type 'GetServoPower-response"
  (cl:format cl:nil "int8 Power~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetServoPower-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetServoPower-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetServoPower-response
    (cl:cons ':Power (Power msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetServoPower)))
  'GetServoPower-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetServoPower)))
  'GetServoPower-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetServoPower)))
  "Returns string type for a service object of type '<GetServoPower>"
  "module_controller/GetServoPower")
