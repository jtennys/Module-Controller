; Auto-generated. Do not edit!


(cl:in-package module_controller-srv)


;//! \htmlinclude GetModuleTwist-request.msg.html

(cl:defclass <GetModuleTwist-request> (roslisp-msg-protocol:ros-message)
  ((ID
    :reader ID
    :initarg :ID
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GetModuleTwist-request (<GetModuleTwist-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetModuleTwist-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetModuleTwist-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name module_controller-srv:<GetModuleTwist-request> is deprecated: use module_controller-srv:GetModuleTwist-request instead.")))

(cl:ensure-generic-function 'ID-val :lambda-list '(m))
(cl:defmethod ID-val ((m <GetModuleTwist-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader module_controller-srv:ID-val is deprecated.  Use module_controller-srv:ID instead.")
  (ID m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetModuleTwist-request>) ostream)
  "Serializes a message object of type '<GetModuleTwist-request>"
  (cl:let* ((signed (cl:slot-value msg 'ID)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetModuleTwist-request>) istream)
  "Deserializes a message object of type '<GetModuleTwist-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ID) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetModuleTwist-request>)))
  "Returns string type for a service object of type '<GetModuleTwist-request>"
  "module_controller/GetModuleTwistRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetModuleTwist-request)))
  "Returns string type for a service object of type 'GetModuleTwist-request"
  "module_controller/GetModuleTwistRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetModuleTwist-request>)))
  "Returns md5sum for a message object of type '<GetModuleTwist-request>"
  "9ef85376904fa5a329707b43ba3623a5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetModuleTwist-request)))
  "Returns md5sum for a message object of type 'GetModuleTwist-request"
  "9ef85376904fa5a329707b43ba3623a5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetModuleTwist-request>)))
  "Returns full string definition for message of type '<GetModuleTwist-request>"
  (cl:format cl:nil "int8 ID~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetModuleTwist-request)))
  "Returns full string definition for message of type 'GetModuleTwist-request"
  (cl:format cl:nil "int8 ID~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetModuleTwist-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetModuleTwist-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetModuleTwist-request
    (cl:cons ':ID (ID msg))
))
;//! \htmlinclude GetModuleTwist-response.msg.html

(cl:defclass <GetModuleTwist-response> (roslisp-msg-protocol:ros-message)
  ((Twist
    :reader Twist
    :initarg :Twist
    :type cl:float
    :initform 0.0))
)

(cl:defclass GetModuleTwist-response (<GetModuleTwist-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetModuleTwist-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetModuleTwist-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name module_controller-srv:<GetModuleTwist-response> is deprecated: use module_controller-srv:GetModuleTwist-response instead.")))

(cl:ensure-generic-function 'Twist-val :lambda-list '(m))
(cl:defmethod Twist-val ((m <GetModuleTwist-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader module_controller-srv:Twist-val is deprecated.  Use module_controller-srv:Twist instead.")
  (Twist m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetModuleTwist-response>) ostream)
  "Serializes a message object of type '<GetModuleTwist-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Twist))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetModuleTwist-response>) istream)
  "Deserializes a message object of type '<GetModuleTwist-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Twist) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetModuleTwist-response>)))
  "Returns string type for a service object of type '<GetModuleTwist-response>"
  "module_controller/GetModuleTwistResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetModuleTwist-response)))
  "Returns string type for a service object of type 'GetModuleTwist-response"
  "module_controller/GetModuleTwistResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetModuleTwist-response>)))
  "Returns md5sum for a message object of type '<GetModuleTwist-response>"
  "9ef85376904fa5a329707b43ba3623a5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetModuleTwist-response)))
  "Returns md5sum for a message object of type 'GetModuleTwist-response"
  "9ef85376904fa5a329707b43ba3623a5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetModuleTwist-response>)))
  "Returns full string definition for message of type '<GetModuleTwist-response>"
  (cl:format cl:nil "float32 Twist~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetModuleTwist-response)))
  "Returns full string definition for message of type 'GetModuleTwist-response"
  (cl:format cl:nil "float32 Twist~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetModuleTwist-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetModuleTwist-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetModuleTwist-response
    (cl:cons ':Twist (Twist msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetModuleTwist)))
  'GetModuleTwist-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetModuleTwist)))
  'GetModuleTwist-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetModuleTwist)))
  "Returns string type for a service object of type '<GetModuleTwist>"
  "module_controller/GetModuleTwist")
