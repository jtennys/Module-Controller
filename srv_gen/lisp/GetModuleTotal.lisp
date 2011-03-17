; Auto-generated. Do not edit!


(cl:in-package module_controller-srv)


;//! \htmlinclude GetModuleTotal-request.msg.html

(cl:defclass <GetModuleTotal-request> (roslisp-msg-protocol:ros-message)
  ((Nothing
    :reader Nothing
    :initarg :Nothing
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GetModuleTotal-request (<GetModuleTotal-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetModuleTotal-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetModuleTotal-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name module_controller-srv:<GetModuleTotal-request> is deprecated: use module_controller-srv:GetModuleTotal-request instead.")))

(cl:ensure-generic-function 'Nothing-val :lambda-list '(m))
(cl:defmethod Nothing-val ((m <GetModuleTotal-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader module_controller-srv:Nothing-val is deprecated.  Use module_controller-srv:Nothing instead.")
  (Nothing m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetModuleTotal-request>) ostream)
  "Serializes a message object of type '<GetModuleTotal-request>"
  (cl:let* ((signed (cl:slot-value msg 'Nothing)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetModuleTotal-request>) istream)
  "Deserializes a message object of type '<GetModuleTotal-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Nothing) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetModuleTotal-request>)))
  "Returns string type for a service object of type '<GetModuleTotal-request>"
  "module_controller/GetModuleTotalRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetModuleTotal-request)))
  "Returns string type for a service object of type 'GetModuleTotal-request"
  "module_controller/GetModuleTotalRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetModuleTotal-request>)))
  "Returns md5sum for a message object of type '<GetModuleTotal-request>"
  "443eae3a81f8e9931eb80c6e96bb1875")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetModuleTotal-request)))
  "Returns md5sum for a message object of type 'GetModuleTotal-request"
  "443eae3a81f8e9931eb80c6e96bb1875")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetModuleTotal-request>)))
  "Returns full string definition for message of type '<GetModuleTotal-request>"
  (cl:format cl:nil "int8 Nothing~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetModuleTotal-request)))
  "Returns full string definition for message of type 'GetModuleTotal-request"
  (cl:format cl:nil "int8 Nothing~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetModuleTotal-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetModuleTotal-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetModuleTotal-request
    (cl:cons ':Nothing (Nothing msg))
))
;//! \htmlinclude GetModuleTotal-response.msg.html

(cl:defclass <GetModuleTotal-response> (roslisp-msg-protocol:ros-message)
  ((Total
    :reader Total
    :initarg :Total
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GetModuleTotal-response (<GetModuleTotal-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetModuleTotal-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetModuleTotal-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name module_controller-srv:<GetModuleTotal-response> is deprecated: use module_controller-srv:GetModuleTotal-response instead.")))

(cl:ensure-generic-function 'Total-val :lambda-list '(m))
(cl:defmethod Total-val ((m <GetModuleTotal-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader module_controller-srv:Total-val is deprecated.  Use module_controller-srv:Total instead.")
  (Total m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetModuleTotal-response>) ostream)
  "Serializes a message object of type '<GetModuleTotal-response>"
  (cl:let* ((signed (cl:slot-value msg 'Total)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetModuleTotal-response>) istream)
  "Deserializes a message object of type '<GetModuleTotal-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Total) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetModuleTotal-response>)))
  "Returns string type for a service object of type '<GetModuleTotal-response>"
  "module_controller/GetModuleTotalResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetModuleTotal-response)))
  "Returns string type for a service object of type 'GetModuleTotal-response"
  "module_controller/GetModuleTotalResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetModuleTotal-response>)))
  "Returns md5sum for a message object of type '<GetModuleTotal-response>"
  "443eae3a81f8e9931eb80c6e96bb1875")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetModuleTotal-response)))
  "Returns md5sum for a message object of type 'GetModuleTotal-response"
  "443eae3a81f8e9931eb80c6e96bb1875")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetModuleTotal-response>)))
  "Returns full string definition for message of type '<GetModuleTotal-response>"
  (cl:format cl:nil "int8 Total~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetModuleTotal-response)))
  "Returns full string definition for message of type 'GetModuleTotal-response"
  (cl:format cl:nil "int8 Total~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetModuleTotal-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetModuleTotal-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetModuleTotal-response
    (cl:cons ':Total (Total msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetModuleTotal)))
  'GetModuleTotal-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetModuleTotal)))
  'GetModuleTotal-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetModuleTotal)))
  "Returns string type for a service object of type '<GetModuleTotal>"
  "module_controller/GetModuleTotal")
