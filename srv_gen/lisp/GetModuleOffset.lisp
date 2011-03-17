; Auto-generated. Do not edit!


(cl:in-package module_controller-srv)


;//! \htmlinclude GetModuleOffset-request.msg.html

(cl:defclass <GetModuleOffset-request> (roslisp-msg-protocol:ros-message)
  ((ID
    :reader ID
    :initarg :ID
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GetModuleOffset-request (<GetModuleOffset-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetModuleOffset-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetModuleOffset-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name module_controller-srv:<GetModuleOffset-request> is deprecated: use module_controller-srv:GetModuleOffset-request instead.")))

(cl:ensure-generic-function 'ID-val :lambda-list '(m))
(cl:defmethod ID-val ((m <GetModuleOffset-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader module_controller-srv:ID-val is deprecated.  Use module_controller-srv:ID instead.")
  (ID m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetModuleOffset-request>) ostream)
  "Serializes a message object of type '<GetModuleOffset-request>"
  (cl:let* ((signed (cl:slot-value msg 'ID)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetModuleOffset-request>) istream)
  "Deserializes a message object of type '<GetModuleOffset-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ID) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetModuleOffset-request>)))
  "Returns string type for a service object of type '<GetModuleOffset-request>"
  "module_controller/GetModuleOffsetRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetModuleOffset-request)))
  "Returns string type for a service object of type 'GetModuleOffset-request"
  "module_controller/GetModuleOffsetRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetModuleOffset-request>)))
  "Returns md5sum for a message object of type '<GetModuleOffset-request>"
  "d0507e8dc19c9273ee2f093c09374bee")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetModuleOffset-request)))
  "Returns md5sum for a message object of type 'GetModuleOffset-request"
  "d0507e8dc19c9273ee2f093c09374bee")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetModuleOffset-request>)))
  "Returns full string definition for message of type '<GetModuleOffset-request>"
  (cl:format cl:nil "int8 ID~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetModuleOffset-request)))
  "Returns full string definition for message of type 'GetModuleOffset-request"
  (cl:format cl:nil "int8 ID~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetModuleOffset-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetModuleOffset-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetModuleOffset-request
    (cl:cons ':ID (ID msg))
))
;//! \htmlinclude GetModuleOffset-response.msg.html

(cl:defclass <GetModuleOffset-response> (roslisp-msg-protocol:ros-message)
  ((Offset
    :reader Offset
    :initarg :Offset
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GetModuleOffset-response (<GetModuleOffset-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetModuleOffset-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetModuleOffset-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name module_controller-srv:<GetModuleOffset-response> is deprecated: use module_controller-srv:GetModuleOffset-response instead.")))

(cl:ensure-generic-function 'Offset-val :lambda-list '(m))
(cl:defmethod Offset-val ((m <GetModuleOffset-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader module_controller-srv:Offset-val is deprecated.  Use module_controller-srv:Offset instead.")
  (Offset m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetModuleOffset-response>) ostream)
  "Serializes a message object of type '<GetModuleOffset-response>"
  (cl:let* ((signed (cl:slot-value msg 'Offset)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetModuleOffset-response>) istream)
  "Deserializes a message object of type '<GetModuleOffset-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Offset) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetModuleOffset-response>)))
  "Returns string type for a service object of type '<GetModuleOffset-response>"
  "module_controller/GetModuleOffsetResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetModuleOffset-response)))
  "Returns string type for a service object of type 'GetModuleOffset-response"
  "module_controller/GetModuleOffsetResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetModuleOffset-response>)))
  "Returns md5sum for a message object of type '<GetModuleOffset-response>"
  "d0507e8dc19c9273ee2f093c09374bee")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetModuleOffset-response)))
  "Returns md5sum for a message object of type 'GetModuleOffset-response"
  "d0507e8dc19c9273ee2f093c09374bee")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetModuleOffset-response>)))
  "Returns full string definition for message of type '<GetModuleOffset-response>"
  (cl:format cl:nil "int16 Offset~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetModuleOffset-response)))
  "Returns full string definition for message of type 'GetModuleOffset-response"
  (cl:format cl:nil "int16 Offset~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetModuleOffset-response>))
  (cl:+ 0
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetModuleOffset-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetModuleOffset-response
    (cl:cons ':Offset (Offset msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetModuleOffset)))
  'GetModuleOffset-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetModuleOffset)))
  'GetModuleOffset-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetModuleOffset)))
  "Returns string type for a service object of type '<GetModuleOffset>"
  "module_controller/GetModuleOffset")
