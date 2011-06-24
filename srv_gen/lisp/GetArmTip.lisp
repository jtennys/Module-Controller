; Auto-generated. Do not edit!


(cl:in-package module_controller-srv)


;//! \htmlinclude GetArmTip-request.msg.html

(cl:defclass <GetArmTip-request> (roslisp-msg-protocol:ros-message)
  ((Nothing
    :reader Nothing
    :initarg :Nothing
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GetArmTip-request (<GetArmTip-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetArmTip-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetArmTip-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name module_controller-srv:<GetArmTip-request> is deprecated: use module_controller-srv:GetArmTip-request instead.")))

(cl:ensure-generic-function 'Nothing-val :lambda-list '(m))
(cl:defmethod Nothing-val ((m <GetArmTip-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader module_controller-srv:Nothing-val is deprecated.  Use module_controller-srv:Nothing instead.")
  (Nothing m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetArmTip-request>) ostream)
  "Serializes a message object of type '<GetArmTip-request>"
  (cl:let* ((signed (cl:slot-value msg 'Nothing)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetArmTip-request>) istream)
  "Deserializes a message object of type '<GetArmTip-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Nothing) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetArmTip-request>)))
  "Returns string type for a service object of type '<GetArmTip-request>"
  "module_controller/GetArmTipRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetArmTip-request)))
  "Returns string type for a service object of type 'GetArmTip-request"
  "module_controller/GetArmTipRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetArmTip-request>)))
  "Returns md5sum for a message object of type '<GetArmTip-request>"
  "be5e429ef59289bd2dccbf657856fbfc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetArmTip-request)))
  "Returns md5sum for a message object of type 'GetArmTip-request"
  "be5e429ef59289bd2dccbf657856fbfc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetArmTip-request>)))
  "Returns full string definition for message of type '<GetArmTip-request>"
  (cl:format cl:nil "int8 Nothing~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetArmTip-request)))
  "Returns full string definition for message of type 'GetArmTip-request"
  (cl:format cl:nil "int8 Nothing~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetArmTip-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetArmTip-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetArmTip-request
    (cl:cons ':Nothing (Nothing msg))
))
;//! \htmlinclude GetArmTip-response.msg.html

(cl:defclass <GetArmTip-response> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (z
    :reader z
    :initarg :z
    :type cl:float
    :initform 0.0))
)

(cl:defclass GetArmTip-response (<GetArmTip-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetArmTip-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetArmTip-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name module_controller-srv:<GetArmTip-response> is deprecated: use module_controller-srv:GetArmTip-response instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <GetArmTip-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader module_controller-srv:x-val is deprecated.  Use module_controller-srv:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <GetArmTip-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader module_controller-srv:y-val is deprecated.  Use module_controller-srv:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <GetArmTip-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader module_controller-srv:z-val is deprecated.  Use module_controller-srv:z instead.")
  (z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetArmTip-response>) ostream)
  "Serializes a message object of type '<GetArmTip-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetArmTip-response>) istream)
  "Deserializes a message object of type '<GetArmTip-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetArmTip-response>)))
  "Returns string type for a service object of type '<GetArmTip-response>"
  "module_controller/GetArmTipResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetArmTip-response)))
  "Returns string type for a service object of type 'GetArmTip-response"
  "module_controller/GetArmTipResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetArmTip-response>)))
  "Returns md5sum for a message object of type '<GetArmTip-response>"
  "be5e429ef59289bd2dccbf657856fbfc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetArmTip-response)))
  "Returns md5sum for a message object of type 'GetArmTip-response"
  "be5e429ef59289bd2dccbf657856fbfc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetArmTip-response>)))
  "Returns full string definition for message of type '<GetArmTip-response>"
  (cl:format cl:nil "float32 x~%float32 y~%float32 z~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetArmTip-response)))
  "Returns full string definition for message of type 'GetArmTip-response"
  (cl:format cl:nil "float32 x~%float32 y~%float32 z~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetArmTip-response>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetArmTip-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetArmTip-response
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetArmTip)))
  'GetArmTip-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetArmTip)))
  'GetArmTip-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetArmTip)))
  "Returns string type for a service object of type '<GetArmTip>"
  "module_controller/GetArmTip")
