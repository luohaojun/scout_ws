; Auto-generated. Do not edit!


(cl:in-package offb-msg)


;//! \htmlinclude obj.msg.html

(cl:defclass <obj> (roslisp-msg-protocol:ros-message)
  ((object
    :reader object
    :initarg :object
    :type cl:boolean
    :initform cl:nil)
   (X_c
    :reader X_c
    :initarg :X_c
    :type cl:float
    :initform 0.0)
   (Y_c
    :reader Y_c
    :initarg :Y_c
    :type cl:float
    :initform 0.0)
   (Z_c
    :reader Z_c
    :initarg :Z_c
    :type cl:float
    :initform 0.0))
)

(cl:defclass obj (<obj>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <obj>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'obj)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name offb-msg:<obj> is deprecated: use offb-msg:obj instead.")))

(cl:ensure-generic-function 'object-val :lambda-list '(m))
(cl:defmethod object-val ((m <obj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader offb-msg:object-val is deprecated.  Use offb-msg:object instead.")
  (object m))

(cl:ensure-generic-function 'X_c-val :lambda-list '(m))
(cl:defmethod X_c-val ((m <obj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader offb-msg:X_c-val is deprecated.  Use offb-msg:X_c instead.")
  (X_c m))

(cl:ensure-generic-function 'Y_c-val :lambda-list '(m))
(cl:defmethod Y_c-val ((m <obj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader offb-msg:Y_c-val is deprecated.  Use offb-msg:Y_c instead.")
  (Y_c m))

(cl:ensure-generic-function 'Z_c-val :lambda-list '(m))
(cl:defmethod Z_c-val ((m <obj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader offb-msg:Z_c-val is deprecated.  Use offb-msg:Z_c instead.")
  (Z_c m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <obj>) ostream)
  "Serializes a message object of type '<obj>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'object) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'X_c))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'Y_c))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'Z_c))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <obj>) istream)
  "Deserializes a message object of type '<obj>"
    (cl:setf (cl:slot-value msg 'object) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'X_c) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Y_c) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Z_c) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<obj>)))
  "Returns string type for a message object of type '<obj>"
  "offb/obj")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'obj)))
  "Returns string type for a message object of type 'obj"
  "offb/obj")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<obj>)))
  "Returns md5sum for a message object of type '<obj>"
  "281cc9156c896ee80119925d1b2feeb6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'obj)))
  "Returns md5sum for a message object of type 'obj"
  "281cc9156c896ee80119925d1b2feeb6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<obj>)))
  "Returns full string definition for message of type '<obj>"
  (cl:format cl:nil "bool object~%float64 X_c ~%float64 Y_c~%float64 Z_c~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'obj)))
  "Returns full string definition for message of type 'obj"
  (cl:format cl:nil "bool object~%float64 X_c ~%float64 Y_c~%float64 Z_c~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <obj>))
  (cl:+ 0
     1
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <obj>))
  "Converts a ROS message object to a list"
  (cl:list 'obj
    (cl:cons ':object (object msg))
    (cl:cons ':X_c (X_c msg))
    (cl:cons ':Y_c (Y_c msg))
    (cl:cons ':Z_c (Z_c msg))
))
