;; Auto-generated. Do not edit!


(when (boundp 'offb::obj)
  (if (not (find-package "OFFB"))
    (make-package "OFFB"))
  (shadow 'obj (find-package "OFFB")))
(unless (find-package "OFFB::OBJ")
  (make-package "OFFB::OBJ"))

(in-package "ROS")
;;//! \htmlinclude obj.msg.html


(defclass offb::obj
  :super ros::object
  :slots (_object _X_c _Y_c _Z_c ))

(defmethod offb::obj
  (:init
   (&key
    ((:object __object) nil)
    ((:X_c __X_c) 0.0)
    ((:Y_c __Y_c) 0.0)
    ((:Z_c __Z_c) 0.0)
    )
   (send-super :init)
   (setq _object __object)
   (setq _X_c (float __X_c))
   (setq _Y_c (float __Y_c))
   (setq _Z_c (float __Z_c))
   self)
  (:object
   (&optional (__object :null))
   (if (not (eq __object :null)) (setq _object __object)) _object)
  (:X_c
   (&optional __X_c)
   (if __X_c (setq _X_c __X_c)) _X_c)
  (:Y_c
   (&optional __Y_c)
   (if __Y_c (setq _Y_c __Y_c)) _Y_c)
  (:Z_c
   (&optional __Z_c)
   (if __Z_c (setq _Z_c __Z_c)) _Z_c)
  (:serialization-length
   ()
   (+
    ;; bool _object
    1
    ;; float64 _X_c
    8
    ;; float64 _Y_c
    8
    ;; float64 _Z_c
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _object
       (if _object (write-byte -1 s) (write-byte 0 s))
     ;; float64 _X_c
       (sys::poke _X_c (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _Y_c
       (sys::poke _Y_c (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _Z_c
       (sys::poke _Z_c (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _object
     (setq _object (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64 _X_c
     (setq _X_c (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _Y_c
     (setq _Y_c (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _Z_c
     (setq _Z_c (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get offb::obj :md5sum-) "281cc9156c896ee80119925d1b2feeb6")
(setf (get offb::obj :datatype-) "offb/obj")
(setf (get offb::obj :definition-)
      "bool object
float64 X_c 
float64 Y_c
float64 Z_c


")



(provide :offb/obj "281cc9156c896ee80119925d1b2feeb6")


