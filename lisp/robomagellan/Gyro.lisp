; Auto-generated. Do not edit!


(in-package robomagellan-msg)


;//! \htmlinclude Gyro.msg.html

(defclass <Gyro> (ros-message)
  ((header
    :reader header-val
    :initarg :header
    :type roslib-msg:<Header>
    :initform (make-instance 'roslib-msg:<Header>))
   (rotationRate
    :reader rotationRate-val
    :initarg :rotationRate
    :type fixnum
    :initform 0)
   (rotationDirection
    :reader rotationDirection-val
    :initarg :rotationDirection
    :type string
    :initform ""))
)
(defmethod serialize ((msg <Gyro>) ostream)
  "Serializes a message object of type '<Gyro>"
  (serialize (slot-value msg 'header) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'rotationRate)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'rotationRate)) ostream)
  (let ((__ros_str_len (length (slot-value msg 'rotationDirection))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'rotationDirection))
)
(defmethod deserialize ((msg <Gyro>) istream)
  "Deserializes a message object of type '<Gyro>"
  (deserialize (slot-value msg 'header) istream)
  (setf (ldb (byte 8 0) (slot-value msg 'rotationRate)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'rotationRate)) (read-byte istream))
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'rotationDirection) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'rotationDirection) __ros_str_idx) (code-char (read-byte istream)))))
  msg
)
(defmethod ros-datatype ((msg (eql '<Gyro>)))
  "Returns string type for a message object of type '<Gyro>"
  "robomagellan/Gyro")
(defmethod md5sum ((type (eql '<Gyro>)))
  "Returns md5sum for a message object of type '<Gyro>"
  "1b2f5fa32673d71386c8f5bec8644277")
(defmethod message-definition ((type (eql '<Gyro>)))
  "Returns full string definition for message of type '<Gyro>"
  (format nil "#~%# Gyro message~%#~%Header header~%#~%# rotationRate is zero when there is no rotation. as~%# the rate of rotation increases, rotationRate increases~%# from 0~%#~%int16 rotationRate~%#~%# L or R~%#~%string rotationDirection~%~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(defmethod serialization-length ((msg <Gyro>))
  (+ 0
     (serialization-length (slot-value msg 'header))
     2
     4 (length (slot-value msg 'rotationDirection))
))
(defmethod ros-message-to-list ((msg <Gyro>))
  "Converts a ROS message object to a list"
  (list '<Gyro>
    (cons ':header (header-val msg))
    (cons ':rotationRate (rotationRate-val msg))
    (cons ':rotationDirection (rotationDirection-val msg))
))
