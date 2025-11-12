#!/usr/bin/env roseus

;; execute with the following launch
;; roslaunch realsense2_camera rs_rgbd.launch
;; roslaunch stereo_image_sandbox people_pose_estimation_d405.launch gui:=false

(ros::roseus-add-msgs "jsk_recognition_msgs")
(require :jedy "package://jedy_bringup/euslisp/jedy-interface.l")
(require :arrow "models/arrow-object.l")

(ros::roseus "jedy_skeleton_ik")

(unless (boundp '*jedy*)
  (jedy-init)
  (setq *robot* *jedy*)
  (send *robot* :reset-pose)
  (send *ri* :angle-vector (send *jedy* :angle-vector))
  )

;; ik scale
(setq *ik-scale* 0.4)

;; camera coordinate frame
(setq *camera-color-optical-frame* (arrow))
(send *camera-color-optical-frame* :newcoords
      (send (send (send (send (send *robot* :HEAD_LINK1_lk) :copy-worldcoords)
                        :translate #f(0 10 50)) :rotate pi/2 :y :local)
            :rotate -pi/2 :z :local))
(send (send *robot* :HEAD_LINK1_lk) :assoc *camera-color-optical-frame*)

;; visualization coords
(setq human-body-coord (make-coords)
      l-shoulder-coord (make-coords)
      r-shoulder-coord (make-coords)
      l-elbow-coord    (make-coords)
      r-elbow-coord    (make-coords)
      l-hand-coord     (make-coords)
      r-hand-coord     (make-coords)
      l-shoulder-target (make-coords)
      r-shoulder-target (make-coords)
      l-hand-target    (make-coords)
      r-hand-target    (make-coords)
      )

;; visualize objects
(objects (list *robot* *camera-color-optical-frame*
               human-body-coord
               l-shoulder-coord r-shoulder-coord
	             l-elbow-coord r-elbow-coord
               l-hand-coord r-hand-coord
               l-shoulder-target r-shoulder-target l-hand-target r-hand-target
	       ))

(defclass Pointing
  :super propertied-object
  :slots ())

(defmethod Pointing
  (:init ()
         ;; topicのSubscriberの設定
         (ros::subscribe "/camera/people_skeleton_with_depth/output/pose"
                         jsk_recognition_msgs::HumanSkeletonArray #'send self :callback 1)
         ) ;; init

  (:callback (msg)
             (setq skeleton (car (send msg :skeletons)))
             (if skeleton
                 (progn
                   (setq bone-names (send skeleton :bone_names)
                         bones      (send skeleton :bones)
                         ;; rs-ls-idx  (position "right_shoulder->left_shoulder" bone-names :test #'equal)
                         rs-ls-idx  (position "left_shoulder->right_shoulder" bone-names :test #'equal)
                         re-rw-idx  (position "right_elbow->right_wrist" bone-names :test #'equal)
                         le-lw-idx  (position "left_elbow->left_wrist" bone-names :test #'equal)
                         )
		   ;; (ros::ros-warn "callback tick")
                   (ros::ros-debug (format nil "~a" bone-names))

                   (when (and rs-ls-idx re-rw-idx le-lw-idx)
                     (setq rs-ls-bone     (elt bones rs-ls-idx) ;; r_shoulder -> l_shoulder
                           ;; r-shoulder-pos (ros::tf-point->pos (send rs-ls-bone :start_point))
                           ;; l-shoulder-pos (ros::tf-point->pos (send rs-ls-bone :end_point))
                           r-shoulder-pos (ros::tf-point->pos (send rs-ls-bone :end_point))
                           l-shoulder-pos (ros::tf-point->pos (send rs-ls-bone :start_point))
                           re-rw-bone     (elt bones re-rw-idx) ;; r_elbow -> r_wrist
                           r-elbow-pos    (ros::tf-point->pos (send re-rw-bone :start_point))
                           r-wrist-pos    (ros::tf-point->pos (send re-rw-bone :end_point))
                           le-lw-bone     (elt bones le-lw-idx) ;; l_elbow -> l_wrist
                           l-elbow-pos    (ros::tf-point->pos (send le-lw-bone :start_point))
                           l-wrist-pos    (ros::tf-point->pos (send le-lw-bone :end_point))
                           )

                     (ros::ros-info (format nil "=============================="))
                     (ros::ros-info (format nil "detected right_wrist at ~a!!" r-wrist-pos))

                     (setq l-shoulder-pos (send *camera-color-optical-frame* :transform-vector l-shoulder-pos)
                           r-shoulder-pos (send *camera-color-optical-frame* :transform-vector r-shoulder-pos)
                           l-elbow-pos    (send *camera-color-optical-frame* :transform-vector l-elbow-pos)
                           r-elbow-pos    (send *camera-color-optical-frame* :transform-vector r-elbow-pos)
                           l-wrist-pos    (send *camera-color-optical-frame* :transform-vector l-wrist-pos)
                           r-wrist-pos    (send *camera-color-optical-frame* :transform-vector r-wrist-pos))

                      ;; ik scale
                      (setq l-shoulder-pos (scale *ik-scale* l-shoulder-pos)
                            r-shoulder-pos (scale *ik-scale* r-shoulder-pos)
                            l-elbow-pos    (scale *ik-scale* l-elbow-pos)
                            r-elbow-pos    (scale *ik-scale* r-elbow-pos)
                            l-wrist-pos    (scale *ik-scale* l-wrist-pos)
                            r-wrist-pos    (scale *ik-scale* r-wrist-pos))

		     ;; calculate human body coord from the shoulder vector
                     (setq rs-ls-vec (v- l-shoulder-pos r-shoulder-pos)
                           y-axis (normalize-vector rs-ls-vec)
                           z-axis #f(0 0 1)
                           x-axis (normalize-vector (v* y-axis z-axis))
                           z-axis (normalize-vector (v* x-axis y-axis)))
                     (send human-body-coord :newcoords (make-coords
                                                        :pos (v+ r-shoulder-pos (scale 0.5 rs-ls-vec))
                                                        :rot (transpose (matrix x-axis y-axis z-axis)))
                           )

                     (send l-shoulder-coord :move-to (make-coords :pos l-shoulder-pos) :world)
                     (send r-shoulder-coord :move-to (make-coords :pos r-shoulder-pos) :world)
                     (send l-elbow-coord    :move-to (make-coords :pos l-elbow-pos) :world)
                     (send r-elbow-coord    :move-to (make-coords :pos r-elbow-pos) :world)
                     (send l-hand-coord     :move-to (make-coords :pos l-wrist-pos) :world)
                     (send r-hand-coord     :move-to (make-coords :pos r-wrist-pos) :world)

                    (send r-shoulder-target :move-to (send human-body-coord :transformation r-shoulder-coord) :world)
                    (send l-shoulder-target :move-to (send human-body-coord :transformation l-shoulder-coord) :world)
                    (send r-hand-target     :move-to (send human-body-coord :transformation r-hand-coord) :world)
                    (send l-hand-target     :move-to (send human-body-coord :transformation l-hand-coord) :world)

                    (setq base-coord (send *robot* :worldcoords))
                    (send r-shoulder-target :transform base-coord :world)
                    (send l-shoulder-target :transform base-coord :world)
                    (send r-hand-target     :transform base-coord :world)
                    (send l-hand-target     :transform base-coord :world)

                     (send self :move-hand l-hand-target r-hand-target)

                     (send *irtviewer* :draw-objects)
                     )
                   )
               )
             ) ;; callback end

  (:move-hand (l-hand r-hand)
              (let (target-coords)
                (send *robot* :angle-vector (send *ri* :potentio-vector))
                (when ;; ik solved
                    (and (send *robot* :rarm :inverse-kinematics r-hand :rotation-axis nil)
                         (send *robot* :larm :inverse-kinematics l-hand :rotation-axis nil))
                  (ros::ros-warn "ik solved")
                  )
                  )
              )
  )

(setq act (instance Pointing :init))

(defun ik-start ()
  (ros::rate 20)
  (do-until-key
   ()
   (x::window-main-one)
   (ros::spin-once)
   (ros::sleep)
   )

  ;; (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
  (unix::usleep (* 2000 1000))
  )
