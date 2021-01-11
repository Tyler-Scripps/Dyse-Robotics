;; Auto-generated. Do not edit!


(when (boundp 'arman_controller::ArmanDriverRequest)
  (if (not (find-package "ARMAN_CONTROLLER"))
    (make-package "ARMAN_CONTROLLER"))
  (shadow 'ArmanDriverRequest (find-package "ARMAN_CONTROLLER")))
(unless (find-package "ARMAN_CONTROLLER::ARMANDRIVERREQUEST")
  (make-package "ARMAN_CONTROLLER::ARMANDRIVERREQUEST"))
(unless (find-package "ARMAN_CONTROLLER::ARMANDRIVERREQUESTREQUEST")
  (make-package "ARMAN_CONTROLLER::ARMANDRIVERREQUESTREQUEST"))
(unless (find-package "ARMAN_CONTROLLER::ARMANDRIVERREQUESTRESPONSE")
  (make-package "ARMAN_CONTROLLER::ARMANDRIVERREQUESTRESPONSE"))

(in-package "ROS")





(defclass arman_controller::ArmanDriverRequestRequest
  :super ros::object
  :slots (_joint0 _joint1 _joint2 _joint3 _joint4 ))

(defmethod arman_controller::ArmanDriverRequestRequest
  (:init
   (&key
    ((:joint0 __joint0) 0.0)
    ((:joint1 __joint1) 0.0)
    ((:joint2 __joint2) 0.0)
    ((:joint3 __joint3) 0.0)
    ((:joint4 __joint4) 0.0)
    )
   (send-super :init)
   (setq _joint0 (float __joint0))
   (setq _joint1 (float __joint1))
   (setq _joint2 (float __joint2))
   (setq _joint3 (float __joint3))
   (setq _joint4 (float __joint4))
   self)
  (:joint0
   (&optional __joint0)
   (if __joint0 (setq _joint0 __joint0)) _joint0)
  (:joint1
   (&optional __joint1)
   (if __joint1 (setq _joint1 __joint1)) _joint1)
  (:joint2
   (&optional __joint2)
   (if __joint2 (setq _joint2 __joint2)) _joint2)
  (:joint3
   (&optional __joint3)
   (if __joint3 (setq _joint3 __joint3)) _joint3)
  (:joint4
   (&optional __joint4)
   (if __joint4 (setq _joint4 __joint4)) _joint4)
  (:serialization-length
   ()
   (+
    ;; float32 _joint0
    4
    ;; float32 _joint1
    4
    ;; float32 _joint2
    4
    ;; float32 _joint3
    4
    ;; float32 _joint4
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _joint0
       (sys::poke _joint0 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _joint1
       (sys::poke _joint1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _joint2
       (sys::poke _joint2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _joint3
       (sys::poke _joint3 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _joint4
       (sys::poke _joint4 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _joint0
     (setq _joint0 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _joint1
     (setq _joint1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _joint2
     (setq _joint2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _joint3
     (setq _joint3 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _joint4
     (setq _joint4 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass arman_controller::ArmanDriverRequestResponse
  :super ros::object
  :slots (_status ))

(defmethod arman_controller::ArmanDriverRequestResponse
  (:init
   (&key
    ((:status __status) 0)
    )
   (send-super :init)
   (setq _status (round __status))
   self)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:serialization-length
   ()
   (+
    ;; int8 _status
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _status
       (write-byte _status s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _status
     (setq _status (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _status 127) (setq _status (- _status 256)))
   ;;
   self)
  )

(defclass arman_controller::ArmanDriverRequest
  :super ros::object
  :slots ())

(setf (get arman_controller::ArmanDriverRequest :md5sum-) "32056f59691abbe99bbd41d6989295c4")
(setf (get arman_controller::ArmanDriverRequest :datatype-) "arman_controller/ArmanDriverRequest")
(setf (get arman_controller::ArmanDriverRequest :request) arman_controller::ArmanDriverRequestRequest)
(setf (get arman_controller::ArmanDriverRequest :response) arman_controller::ArmanDriverRequestResponse)

(defmethod arman_controller::ArmanDriverRequestRequest
  (:response () (instance arman_controller::ArmanDriverRequestResponse :init)))

(setf (get arman_controller::ArmanDriverRequestRequest :md5sum-) "32056f59691abbe99bbd41d6989295c4")
(setf (get arman_controller::ArmanDriverRequestRequest :datatype-) "arman_controller/ArmanDriverRequestRequest")
(setf (get arman_controller::ArmanDriverRequestRequest :definition-)
      "float32 joint0
float32 joint1
float32 joint2
float32 joint3
float32 joint4
---
int8 status

")

(setf (get arman_controller::ArmanDriverRequestResponse :md5sum-) "32056f59691abbe99bbd41d6989295c4")
(setf (get arman_controller::ArmanDriverRequestResponse :datatype-) "arman_controller/ArmanDriverRequestResponse")
(setf (get arman_controller::ArmanDriverRequestResponse :definition-)
      "float32 joint0
float32 joint1
float32 joint2
float32 joint3
float32 joint4
---
int8 status

")



(provide :arman_controller/ArmanDriverRequest "32056f59691abbe99bbd41d6989295c4")


