(load "package://jsk_maps/src/building-model.l")

(load "models/elevator_call_panel-object.l")
(load "models/elevator_inside_panel-object.l")

(load "models/room602-scene.l")
(load "models/room610-scene.l")

(defun make-eng8-b1f-scene ()
  (let* (scene map floor-names rooms spots objects)
    (setq map (make-floor-plane (ros::resolve-ros-path "package://jsk_maps/raw_maps/eng8-b1f-0.05.pgm") 50 -7500 -75000 :name "/eng8/b1f/floor-plane"))

    ;; B1F
    (make-cascoords-tree
     map
     '(:rot #2f((0.0 1.0 0.0) (-1.0 0.0 0.0) (0.0 0.0 1.0))
	    :pos #f(3200.0 -29200.0 0.0)
	    :name "/eng8/b1f/elevator_call_panel-front")
     '(:rot #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))
	    :pos #f(4100.0 -31300.0 0.0)
	    :name "/eng8/b1f/elevator_inside_panel-front")
     '(:rot #2f((0.0 -1.0 0.0) (1.0 0.0 0.0) (0.0 0.0 1.0))
	    :pos #f(4000.0 -29000.0 0.0)
	    :name "/eng8/b1f/elevator-outside")
     '(:pos #f(3100 -29900 920) :rpy #f(1.5706 0 0)
	    :name "/eng8/b1f/elevator_call_panel")
     '(:pos #f(4850 -31550 1000) :rpy #f(3.1416 0 0)
	    :name "/eng8/b1f/elevator_inside_panel"))

    (setq scene
	  (instance nav-scene-model :init :name "/eng8/b1f"
		    :objects (append (list map) (send map :descendants))))
    (send scene :map map)
    scene ))


(defun make-eng8-1f-scene ()
  (let* (scene map floor-names rooms spots objects)
    (setq map (make-floor-plane (ros::resolve-ros-path "package://jsk_maps/raw_maps/eng8-1f-0.05.pgm") 50 -8000 -70000 :name "/eng8/1f/floor-plane"))

    ;; 1F
    (make-cascoords-tree
     map
     '(:rot #2f((0.0 1.0 0.0) (-1.0 0.0 0.0) (0.0 0.0 1.0))
	    :pos #f(3200.0 -29100.0 0.0)
	    :name "/eng8/1f/elevator_call_panel-front")
     '(:rot #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))
	    :pos #f(4100.0 -31300.0 0.0)
	    :name "/eng8/1f/elevator_inside_panel-front")
     '(:rot #2f((0.0 -1.0 0.0) (1.0 0.0 0.0) (0.0 0.0 1.0))
	    :pos #f(4000.0 -29000.0 0.0)
	    :name "/eng8/1f/elevator-outside")
     '(:rot #2f((0.0 1.0 0.0) (-1.0 0.0 0.0) (0.0 0.0 1.0))
	    :pos #f(15500.0 -33000.0 0.0)
	    :name "/eng8/1f/pamphlet-front")
     '(:rot #2f((-1.0 0.0 0.0) (0.0 -1.0 0.0) (0.0 0.0 1.0))
	    :pos #f(-1170.0 -27600.0 0.0)
	    :name "/eng8/1f/student-affairs")
     '(:pos #f(3100 -29900 920) :rpy #f(1.5706 0 0)
	    :name "/eng8/1f/elevator_call_panel")
     '(:pos #f(4850 -31550 1000) :rpy #f(3.1416 0 0)
	    :name "/eng8/1f/elevator_inside_panel"))

    (setq scene
	  (instance nav-scene-model :init :name "/eng8/1f"
		    :objects (append (list map) (send map :descendants))))
    (send scene :map map)
    scene ))

(defun make-eng8-6f-scene ()
  (let* (scene map floor-names rooms spots objects room602 room610)
    (setq map (make-floor-plane (ros::resolve-ros-path "package://jsk_maps/raw_maps/eng8-6f-0.05.pgm") 50 -8000 -70000 :name "/eng8/6f/floor-plane"))

    ;; 6F
    (setq room610 (instance room610-scene :init :name "610")
          room602 (instance room602-scene :init :name "602"))
    (setf (get room602 :knowrob-type) "OfficeRoom")
    (setf (get room610 :knowrob-type) "OfficeRoom")
      ;;
    (send room602 :locate (float-vector 0 0 0))
    ;;(send room610 :locate (float-vector 100 (- 3050 300 16322) 0))
    (send room610 :locate (float-vector 100 -13678 0))
    ;;
    (dolist (obj (send room602 :objects)) (send room602 :assoc obj))
    (dolist (obj (send room610 :objects)) (send room610 :assoc obj))
    (send map :assoc room602)
    (send map :assoc room610)

    ;; temporarily copy from jsk_2013_04_pr2_610/euslisp/setup.l
    (setq table-front (make-coords :pos #f(4700 1000 0) :rpy (float-vector -pi/2 0 0)))
    (setq chair-back (make-coords :pos #f(3800 0 0)))
    (setq laundry-front (make-coords :pos #f(2309.927 1601.466 0)))

    ;; convert to map coordinates
    (setq table-front (send table-front :transform room610 :world))
    (setq chair-back  (send chair-back :transform room610 :world))
    (setq laundry-front (send laundry-front :transform room610 :world))

    (make-cascoords-tree
     map
     '(:rot #2f((-1.0 0.0 0.0) (0.0 -1.0 0.0) (0.0 0.0 1.0))
	    :pos #f(7500.0 2300.0 0.0)
	    :name "/eng8/6f/room602-kitchen")
     '(:rot #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))
	    :pos #f(7600.0 2700.0 0.0)
	    :name "/eng8/6f/room602-microwave")
     '(:rot #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))
	    :pos #f(7600.0 1100.0 0.0)
	    :name "/eng8/6f/room602-refrigerator")
     '(:rot #2f((0.0 -1.0 0.0) (1.0 0.0 0.0) (0.0 0.0 1.0))
	    :pos #f(4000.0 4300.0 0.0)
	    :name "/eng8/6f/room602-shelf")
     '(:rot #2f((-0.866025 -0.5 0.0) (0.5 -0.866025 0.0) (0.0 0.0 1.0))
	    :pos #f(1300.0 3800.0 0.0)
	    :name "/eng8/6f/room602-washingmachine")
     '(:rot #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))
	    :pos #f(1750.0 850.0 0.0)
	    :name "/eng8/6f/room602-base")
     '(:rot #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))
	    :pos #f(-400.0 100.0 0.0)
	    :name "/eng8/6f/room602-front")
     '(:rot #2f((0.0 1.0 0.0) (-1.0 0.0 0.0) (0.0 0.0 1.0))
	    :pos #f(3200.0 -29100.0 0.0)
	    :name "/eng8/6f/elevator_call_panel-front")
     '(:rot #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))
	    :pos #f(4100.0 -31300.0 0.0)
	    :name "/eng8/6f/elevator_inside_panel-front")
     '(:rot #2f((0.0 -1.0 0.0) (1.0 0.0 0.0) (0.0 0.0 1.0))
	    :pos #f(4000.0 -29000.0 0.0)
	    :name "/eng8/6f/elevator-outside")
     '(:pos #f(3100 -29900 920) :rpy #f(1.5706 0 0)
	    :name "/eng8/6f/elevator_call_panel")
     '(:pos #f(4850 -31550 1000) :rpy #f(3.1416 0 0)
	    :name "/eng8/6f/elevator_inside_panel")

     ;;start_map_eng8.launch 0  -13.678 0
     '(:rot #2f((-1.0 0.0 0.0) (0.0 -1.0 0.0) (0.0 0.0 1.0))
	    :pos #f(1050.0 -13678.0 0.0)
	    :name "/eng8/6f/room610-base")
     `(:rot ,(quaternion2matrix #f(0.999922619095 0.0 0.0 0.0124400893165))
	    :pos #f(7320.33605375 -13251.8824822 0.0)
	    :name "/eng8/6f/room610-kitchen-front")
     '(:rot #2f((0.417244 0.908794 0.0) (-0.908794 0.417244 0.0) (0.0 0.0 1.0))
	    :pos #f(7286.1672755 -15075.1147252 0.0)
	    :name "/eng8/6f/room610-hsr099-dock")
     `(:rot ,(quaternion2matrix #f(0.720 0.000 0.000 0.694))
	    :pos #f(7242.0 -12676.0 0.0)
	    :name "/eng8/6f/room610-wrs-shelf")
     `(:rot ,(quaternion2matrix #f(0.752 0.000 0.000 -0.659))
	    :pos #f(6661 -15469 0.000)
	    :name "/eng8/6f/room610-trash")
     `(:rot ,(quaternion2matrix #f(0.72138502912 0 0 0.692534215589))
	    :pos #f(5392.84256834 -15084.2744762 0.0)
	    :name "/eng8/6f/room610-table-side")
     `(:rot  ,(send table-front :worldrot)
             :pos ,(send table-front :worldpos)
             :name "/eng8/6f/room610-table-front")
     `(:rot ,(quaternion2matrix #f(-0.00147859882494 0.0 0.0 0.999998906872))
            :pos #f(4938.96524845 -14443.8345583 0.0)
            :name "/eng8/6f/room610-room-center")
     `(:rot ,(quaternion2matrix #f(0.999922619095 0.0 0.0 0.0124400893165))
	    :pos #f(7356.0877801 -13793.4295069 0.0)
	    :name "/eng8/6f/room610-kitchen-front2")
     `(:rot ,(quaternion2matrix #f(0.726516499466 0.0 0.0 -0.687149020231))
            :pos #f(5730.35883839 -15699.9261594 0.0)
            :name "/eng8/6f/room610-working-table-01")
     `(:rot  ,(send chair-back :worldrot)
             :pos ,(send chair-back :worldpos)
             :name "/eng8/6f/room610-chair-back")
     `(:rot  ,(send laundry-front :worldrot)
             :pos ,(send laundry-front :worldpos)
             :name "/eng8/6f/room610-laundry-flont")
     )
    
    (setq scene
	  (instance nav-scene-model :init :name "/eng8/6f"
		    :objects (append (list map) (send map :descendants))))
    (send scene :map map)
    scene ))


(defun make-eng8-scene ()
  (let* (scene maps floor-names spots objects)
    (setq maps (list (make-eng8-b1f-scene)
		     (make-eng8-1f-scene)
		     (make-eng8-6f-scene)))

    ;; this is not true
    (dotimes (i (length maps))
      (send (elt maps i) :translate (float-vector 0 0 (* i 10000))))


    (setq scene
	  (instance
	   building-model :init :name "/eng8"
	   :objects
	   (append
	    maps
	    (list
	     (elevator_call_panel :name "elevator_call_panel")
	     (elevator_inside_panel :name "elevator_inside_panel"))
	    )))

    ;; scene -> map
    ;;(send scene :assoc map)
    ;;(send scene :floors floor-names) ;; set floor information

    ;; visualize region
    (send-all (send-all maps :map) :set-color #f(0 1 0 0.3))

    (dolist (obj (send scene :objects))
      ;; first charactor of obj-name must be slash
      (when (not (eq (char (send obj :name) 0) #\/))
	(send obj :name (format nil "/~A" (send obj :name)))))

    scene))

;; example
;; (load "package://pr2eus/pr2-interface")
;; (ros::roseus "eng8_navigation")
;; (defvar *tfl* (instance ros::transform-listener :init))
;; (setq *ri* (instance pr2-interface :init))
;; (objects (list (setq *eng8-scene*  (make-eng8-scene))))
;; (move-to (car (send *eng8-scene* :find-object "/eng8/6f/room602-front")))
;;
;; debug
#|
(defun move-to-test (&optional (spot "/eng8/6f/room602-base"))
  (load "eng8-map-scene.l")
  (objects (list (setq *eng8-scene*  (make-eng8-scene))))
  (send *irtviewer* :look-all (make-cube 1000 1000 10000))
  (send (car (send *eng8-scene* :find-object spot)) :draw-on :flush t :size 1000 :color #f(1 0 0))
  (move-to (car (send *eng8-scene* :find-object spot)))
  )
|#
