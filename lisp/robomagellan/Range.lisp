; Auto-generated. Do not edit!


(in-package robomagellan-msg)


;//! \htmlinclude Range.msg.html

(defclass <Range> (ros-message)
  ((header
    :reader header-val
    :initarg :header
    :type roslib-msg:<Header>
    :initform (make-instance 'roslib-msg:<Header>))
   (rangeInCm
    :reader rangeInCm-val
    :initarg :rangeInCm
    :type fixnum
    :initform 0))
)
(defmethod serialize ((msg <Range>) ostream)
  "Serializes a message object of type '<Range>"
  (serialize (slot-value msg 'header) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'rangeInCm)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'rangeInCm)) ostream)
)
(defmethod deserialize ((msg <Range>) istream)
  "Deserializes a message object of type '<Range>"
  (deserialize (slot-value msg 'header) istream)
  (setf (ldb (byte 8 0) (slot-value msg 'rangeInCm)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'rangeInCm)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<Range>)))
  "Returns string type for a message object of type '<Range>"
  "robomagellan/Range")
(defmethod md5sum ((type (eql '<Range>)))
  "Returns md5sum for a message object of type '<Range>"
  "221595241c26574c794493f2090a3a44")
(defmethod message-definition ((type (eql '<Range>)))
  "Returns full string definition for message of type '<Range>"
  (format nil "Header header~%int16 rangeInCm~%~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(defmethod serialization-length ((msg <Range>))
  (+ 0
     (serialization-length (slot-value msg 'header))
     2
))
(defmethod ros-message-to-list ((msg <Range>))
  "Converts a ROS message object to a list"
  (list '<Range>
    (cons ':header (header-val msg))
    (cons ':rangeInCm (rangeInCm-val msg))
))
