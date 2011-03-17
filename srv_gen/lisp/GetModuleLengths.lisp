; Auto-generated. Do not edit!


(cl:in-package module_controller-srv)


;//! \htmlinclude GetModuleLengths-request.msg.html

(cl:defclass <GetModuleLengths-request> (roslisp-msg-protocol:ros-message)
  ((ID
    :reader ID
    :initarg :ID
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GetModuleLengths-request (<GetModuleLengths-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetModuleLengths-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetModuleLengths-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name module_controller-srv:<GetModuleLengths-request> is deprecated: use module_controller-srv:GetModuleLengths-request instead.")))

(cl:ensure-generic-function 'ID-val :lambda-list '(m))
(cl:defmethod ID-val ((m <GetModuleLengths-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader module_controller-srv:ID-val is deprecated.  Use module_controller-srv:ID instead.")
  (ID m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetModuleLengths-request>) ostream)
  "Serializes a message object of type '<GetModuleLengths-request>"
  (cl:let* ((signed (cl:slot-value msg 'ID)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetModuleLengths-request>) istream)
  "Deserializes a message object of type '<GetModuleLengths-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ID) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetModuleLengths-request>)))
  "Returns string type for a service object of type '<GetModuleLengths-request>"
  "module_controller/GetModuleLengthsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetModuleLengths-request)))
  "Returns string type for a service object of type 'GetModuleLengths-request"
  "module_controller/GetModuleLengthsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetModuleLengths-request>)))
  "Returns md5sum for a message object of type '<GetModuleLengths-request>"
  "4dedf9a1defa9906f28f1326090fd468")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetModuleLengths-request)))
  "Returns md5sum for a message object of type 'GetModuleLengths-request"
  "4dedf9a1defa9906f28f1326090fd468")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetModuleLengths-request>)))
  "Returns full string definition for message of type '<GetModuleLengths-request>"
  (cl:format cl:nil "int8 ID~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetModuleLengths-request)))
  "Returns full string definition for message of type 'GetModuleLengths-request"
  (cl:format cl:nil "int8 ID~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetModuleLengths-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetModuleLengths-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetModuleLengths-request
    (cl:cons ':ID (ID msg))
))
;//! \htmlinclude GetModuleLengths-response.msg.html

(cl:defclass <GetModuleLengths-response> (roslisp-msg-protocol:ros-message)
  ((Upstream
    :reader Upstream
    :initarg :Upstream
    :type cl:float
    :initform 0.0)
   (Downstream
    :reader Downstream
    :initarg :Downstream
    :type cl:float
    :initform 0.0))
)

(cl:defclass GetModuleLengths-response (<GetModuleLengths-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetModuleLengths-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetModuleLengths-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name module_controller-srv:<GetModuleLengths-response> is deprecated: use module_controller-srv:GetModuleLengths-response instead.")))

(cl:ensure-generic-function 'Upstream-val :lambda-list '(m))
(cl:defmethod Upstream-val ((m <GetModuleLengths-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader module_controller-srv:Upstream-val is deprecated.  Use module_controller-srv:Upstream instead.")
  (Upstream m))

(cl:ensure-generic-function 'Downstream-val :lambda-list '(m))
(cl:defmethod Downstream-val ((m <GetModuleLengths-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader module_controller-srv:Downstream-val is deprecated.  Use module_controller-srv:Downstream instead.")
  (Downstream m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetModuleLengths-response>) ostream)
  "Serializes a message object of type '<GetModuleLengths-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Upstream))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Downstream))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetModuleLengths-response>) istream)
  "Deserializes a message object of type '<GetModuleLengths-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Upstream) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Downstream) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetModuleLengths-response>)))
  "Returns string type for a service object of type '<GetModuleLengths-response>"
  "module_controller/GetModuleLengthsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetModuleLengths-response)))
  "Returns string type for a service object of type 'GetModuleLengths-response"
  "module_controller/GetModuleLengthsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetModuleLengths-response>)))
  "Returns md5sum for a message object of type '<GetModuleLengths-response>"
  "4dedf9a1defa9906f28f1326090fd468")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetModuleLengths-response)))
  "Returns md5sum for a message object of type 'GetModuleLengths-response"
  "4dedf9a1defa9906f28f1326090fd468")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetModuleLengths-response>)))
  "Returns full string definition for message of type '<GetModuleLengths-response>"
  (cl:format cl:nil "float32 Upstream~%float32 Downstream~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetModuleLengths-response)))
  "Returns full string definition for message of type 'GetModuleLengths-response"
  (cl:format cl:nil "float32 Upstream~%float32 Downstream~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetModuleLengths-response>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetModuleLengths-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetModuleLengths-response
    (cl:cons ':Upstream (Upstream msg))
    (cl:cons ':Downstream (Downstream msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetModuleLengths)))
  'GetModuleLengths-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetModuleLengths)))
  'GetModuleLengths-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetModuleLengths)))
  "Returns string type for a service object of type '<GetModuleLengths>"
  "module_controller/GetModuleLengths")
