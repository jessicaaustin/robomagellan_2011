; Auto-generated. Do not edit!


(in-package robomagellan-msg)


;//! \htmlinclude Move.msg.html

(defclass <Move> (ros-message)
  ((header
    :reader header-val
    :initarg :header
    :type roslib-msg:<Header>
    :initform (make-instance 'roslib-msg:<Header>))
   (leftWheel
    :reader leftWheel-val
    :initarg :leftWheel
    :type fixnum
    :initform 0)
   (rightWheel
    :reader rightWheel-val
    :initarg :rightWheel
    :type fixnum
    :initform 0))
)
(defmethod serialize ((msg <Move>) ostream)
  "Serializes a message object of type '<Move>"
  (serialize (slot-value msg 'header) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'leftWheel)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'leftWheel)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'rightWheel)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'rightWheel)) ostream)
)
(defmethod deserialize ((msg <Move>) istream)
  "Deserializes a message object of type '<Move>"
  (deserialize (slot-value msg 'header) istream)
  (setf (ldb (byte 8 0) (slot-value msg 'leftWheel)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'leftWheel)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'rightWheel)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'rightWheel)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<Move>)))
  "Returns string type for a message object of type '<Move>"
  "robomagellan/Move")
(defmethod md5sum ((type (eql '<Move>)))
  "Returns md5sum for a message object of type '<Move>"
  "89c351fd26199978a8133cec7430c83a")
(defmethod message-definition ((type (eql '<Move>)))
  "Returns full string definition for message of type '<Move>"
  (format nil "Header header~%int16 leftWheel~%int16 rightWheel~%~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(defmethod serialization-length ((msg <Move>))
  (+ 0
     (serialization-length (slot-value msg 'header))
     2
     2
))
(defmethod ros-message-to-list ((msg <Move>))
  "Converts a ROS message object to a list"
  (list '<Move>
    (cons ':header (header-val msg))
    (cons ':leftWheel (leftWheel-val msg))
    (cons ':rightWheel (rightWheel-val msg))
))
