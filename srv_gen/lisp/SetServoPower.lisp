; Auto-generated. Do not edit!


(cl:in-package module_controller-srv)


;//! \htmlinclude SetServoPower-request.msg.html

(cl:defclass <SetServoPower-request> (roslisp-msg-protocol:ros-message)
  ((ID
    :reader ID
    :initarg :ID
    :type cl:fixnum
    :initform 0)
   (Power
    :reader Power
    :initarg :Power
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SetServoPower-request (<SetServoPower-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetServoPower-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetServoPower-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name module_controller-srv:<SetServoPower-request> is deprecated: use module_controller-srv:SetServoPower-request instead.")))

(cl:ensure-generic-function 'ID-val :lambda-list '(m))
(cl:defmethod ID-val ((m <SetServoPower-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader module_controller-srv:ID-val is deprecated.  Use module_controller-srv:ID instead.")
  (ID m))

(cl:ensure-generic-function 'Power-val :lambda-list '(m))
(cl:defmethod Power-val ((m <SetServoPower-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader module_controller-srv:Power-val is deprecated.  Use module_controller-srv:Power instead.")
  (Power m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetServoPower-request>) ostream)
  "Serializes a message object of type '<SetServoPower-request>"
  (cl:let* ((signed (cl:slot-value msg 'ID)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'Power)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetServoPower-request>) istream)
  "Deserializes a message object of type '<SetServoPower-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ID) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Power) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetServoPower-request>)))
  "Returns string type for a service object of type '<SetServoPower-request>"
  "module_controller/SetServoPowerRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetServoPower-request)))
  "Returns string type for a service object of type 'SetServoPower-request"
  "module_controller/SetServoPowerRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetServoPower-request>)))
  "Returns md5sum for a message object of type '<SetServoPower-request>"
  "06dffc0971a8a6f261fa12c9016dc041")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetServoPower-request)))
  "Returns md5sum for a message object of type 'SetServoPower-request"
  "06dffc0971a8a6f261fa12c9016dc041")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetServoPower-request>)))
  "Returns full string definition for message of type '<SetServoPower-request>"
  (cl:format cl:nil "int8 ID~%int8 Power~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetServoPower-request)))
  "Returns full string definition for message of type 'SetServoPower-request"
  (cl:format cl:nil "int8 ID~%int8 Power~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetServoPower-request>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetServoPower-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetServoPower-request
    (cl:cons ':ID (ID msg))
    (cl:cons ':Power (Power msg))
))
;//! \htmlinclude SetServoPower-response.msg.html

(cl:defclass <SetServoPower-response> (roslisp-msg-protocol:ros-message)
  ((Success
    :reader Success
    :initarg :Success
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SetServoPower-response (<SetServoPower-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetServoPower-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetServoPower-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name module_controller-srv:<SetServoPower-response> is deprecated: use module_controller-srv:SetServoPower-response instead.")))

(cl:ensure-generic-function 'Success-val :lambda-list '(m))
(cl:defmethod Success-val ((m <SetServoPower-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader module_controller-srv:Success-val is deprecated.  Use module_controller-srv:Success instead.")
  (Success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetServoPower-response>) ostream)
  "Serializes a message object of type '<SetServoPower-response>"
  (cl:let* ((signed (cl:slot-value msg 'Success)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetServoPower-response>) istream)
  "Deserializes a message object of type '<SetServoPower-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Success) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetServoPower-response>)))
  "Returns string type for a service object of type '<SetServoPower-response>"
  "module_controller/SetServoPowerResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetServoPower-response)))
  "Returns string type for a service object of type 'SetServoPower-response"
  "module_controller/SetServoPowerResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetServoPower-response>)))
  "Returns md5sum for a message object of type '<SetServoPower-response>"
  "06dffc0971a8a6f261fa12c9016dc041")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetServoPower-response)))
  "Returns md5sum for a message object of type 'SetServoPower-response"
  "06dffc0971a8a6f261fa12c9016dc041")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetServoPower-response>)))
  "Returns full string definition for message of type '<SetServoPower-response>"
  (cl:format cl:nil "int8 Success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetServoPower-response)))
  "Returns full string definition for message of type 'SetServoPower-response"
  (cl:format cl:nil "int8 Success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetServoPower-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetServoPower-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetServoPower-response
    (cl:cons ':Success (Success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetServoPower)))
  'SetServoPower-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetServoPower)))
  'SetServoPower-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetServoPower)))
  "Returns string type for a service object of type '<SetServoPower>"
  "module_controller/SetServoPower")
