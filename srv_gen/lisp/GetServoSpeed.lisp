; Auto-generated. Do not edit!


(cl:in-package module_controller-srv)


;//! \htmlinclude GetServoSpeed-request.msg.html

(cl:defclass <GetServoSpeed-request> (roslisp-msg-protocol:ros-message)
  ((ID
    :reader ID
    :initarg :ID
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GetServoSpeed-request (<GetServoSpeed-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetServoSpeed-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetServoSpeed-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name module_controller-srv:<GetServoSpeed-request> is deprecated: use module_controller-srv:GetServoSpeed-request instead.")))

(cl:ensure-generic-function 'ID-val :lambda-list '(m))
(cl:defmethod ID-val ((m <GetServoSpeed-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader module_controller-srv:ID-val is deprecated.  Use module_controller-srv:ID instead.")
  (ID m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetServoSpeed-request>) ostream)
  "Serializes a message object of type '<GetServoSpeed-request>"
  (cl:let* ((signed (cl:slot-value msg 'ID)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetServoSpeed-request>) istream)
  "Deserializes a message object of type '<GetServoSpeed-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ID) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetServoSpeed-request>)))
  "Returns string type for a service object of type '<GetServoSpeed-request>"
  "module_controller/GetServoSpeedRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetServoSpeed-request)))
  "Returns string type for a service object of type 'GetServoSpeed-request"
  "module_controller/GetServoSpeedRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetServoSpeed-request>)))
  "Returns md5sum for a message object of type '<GetServoSpeed-request>"
  "86b923d1c96b0ad1a92b39bae3ff7fad")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetServoSpeed-request)))
  "Returns md5sum for a message object of type 'GetServoSpeed-request"
  "86b923d1c96b0ad1a92b39bae3ff7fad")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetServoSpeed-request>)))
  "Returns full string definition for message of type '<GetServoSpeed-request>"
  (cl:format cl:nil "int8 ID~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetServoSpeed-request)))
  "Returns full string definition for message of type 'GetServoSpeed-request"
  (cl:format cl:nil "int8 ID~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetServoSpeed-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetServoSpeed-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetServoSpeed-request
    (cl:cons ':ID (ID msg))
))
;//! \htmlinclude GetServoSpeed-response.msg.html

(cl:defclass <GetServoSpeed-response> (roslisp-msg-protocol:ros-message)
  ((Speed
    :reader Speed
    :initarg :Speed
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GetServoSpeed-response (<GetServoSpeed-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetServoSpeed-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetServoSpeed-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name module_controller-srv:<GetServoSpeed-response> is deprecated: use module_controller-srv:GetServoSpeed-response instead.")))

(cl:ensure-generic-function 'Speed-val :lambda-list '(m))
(cl:defmethod Speed-val ((m <GetServoSpeed-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader module_controller-srv:Speed-val is deprecated.  Use module_controller-srv:Speed instead.")
  (Speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetServoSpeed-response>) ostream)
  "Serializes a message object of type '<GetServoSpeed-response>"
  (cl:let* ((signed (cl:slot-value msg 'Speed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetServoSpeed-response>) istream)
  "Deserializes a message object of type '<GetServoSpeed-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Speed) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetServoSpeed-response>)))
  "Returns string type for a service object of type '<GetServoSpeed-response>"
  "module_controller/GetServoSpeedResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetServoSpeed-response)))
  "Returns string type for a service object of type 'GetServoSpeed-response"
  "module_controller/GetServoSpeedResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetServoSpeed-response>)))
  "Returns md5sum for a message object of type '<GetServoSpeed-response>"
  "86b923d1c96b0ad1a92b39bae3ff7fad")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetServoSpeed-response)))
  "Returns md5sum for a message object of type 'GetServoSpeed-response"
  "86b923d1c96b0ad1a92b39bae3ff7fad")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetServoSpeed-response>)))
  "Returns full string definition for message of type '<GetServoSpeed-response>"
  (cl:format cl:nil "int8 Speed~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetServoSpeed-response)))
  "Returns full string definition for message of type 'GetServoSpeed-response"
  (cl:format cl:nil "int8 Speed~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetServoSpeed-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetServoSpeed-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetServoSpeed-response
    (cl:cons ':Speed (Speed msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetServoSpeed)))
  'GetServoSpeed-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetServoSpeed)))
  'GetServoSpeed-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetServoSpeed)))
  "Returns string type for a service object of type '<GetServoSpeed>"
  "module_controller/GetServoSpeed")
