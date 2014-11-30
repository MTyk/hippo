; Auto-generated. Do not edit!


(cl:in-package hippo2-srv)


;//! \htmlinclude Int-request.msg.html

(cl:defclass <Int-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Int-request (<Int-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Int-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Int-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hippo2-srv:<Int-request> is deprecated: use hippo2-srv:Int-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Int-request>) ostream)
  "Serializes a message object of type '<Int-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Int-request>) istream)
  "Deserializes a message object of type '<Int-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Int-request>)))
  "Returns string type for a service object of type '<Int-request>"
  "hippo2/IntRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Int-request)))
  "Returns string type for a service object of type 'Int-request"
  "hippo2/IntRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Int-request>)))
  "Returns md5sum for a message object of type '<Int-request>"
  "19aac5e823802d733295ea3ec20e6350")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Int-request)))
  "Returns md5sum for a message object of type 'Int-request"
  "19aac5e823802d733295ea3ec20e6350")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Int-request>)))
  "Returns full string definition for message of type '<Int-request>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Int-request)))
  "Returns full string definition for message of type 'Int-request"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Int-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Int-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Int-request
))
;//! \htmlinclude Int-response.msg.html

(cl:defclass <Int-response> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:integer
    :initform 0))
)

(cl:defclass Int-response (<Int-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Int-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Int-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hippo2-srv:<Int-response> is deprecated: use hippo2-srv:Int-response instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <Int-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hippo2-srv:x-val is deprecated.  Use hippo2-srv:x instead.")
  (x m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Int-response>) ostream)
  "Serializes a message object of type '<Int-response>"
  (cl:let* ((signed (cl:slot-value msg 'x)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Int-response>) istream)
  "Deserializes a message object of type '<Int-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'x) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Int-response>)))
  "Returns string type for a service object of type '<Int-response>"
  "hippo2/IntResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Int-response)))
  "Returns string type for a service object of type 'Int-response"
  "hippo2/IntResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Int-response>)))
  "Returns md5sum for a message object of type '<Int-response>"
  "19aac5e823802d733295ea3ec20e6350")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Int-response)))
  "Returns md5sum for a message object of type 'Int-response"
  "19aac5e823802d733295ea3ec20e6350")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Int-response>)))
  "Returns full string definition for message of type '<Int-response>"
  (cl:format cl:nil "int32 x~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Int-response)))
  "Returns full string definition for message of type 'Int-response"
  (cl:format cl:nil "int32 x~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Int-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Int-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Int-response
    (cl:cons ':x (x msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Int)))
  'Int-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Int)))
  'Int-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Int)))
  "Returns string type for a service object of type '<Int>"
  "hippo2/Int")