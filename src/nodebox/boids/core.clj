(ns nodebox-boids)

(defrecord boid [x y z velocity target sight space])
(defrecord flock [x y width height depth boids])

(defn to-degrees [radians]
  (/ (* radians 180) Math/PI))

(defn make-boid
  ([] (make-boid 0 0 0))
  ([x y z] (make-boid x y z 70 30))
  ([x y z sight space]
    (let
      [velocity [(- (* (rand) 2) 1) (- (* (rand) 2) 1) (- (* (rand) 2) 1)]]
      (boid. x y z velocity nil sight space))))

(defn make-flock
  ([amount x y width height]
    (make-flock amount x y width height 100.0))
  ([amount x y width height depth]
    (let
      [boids (take amount
               (repeatedly
                 (fn [] (make-boid (+ x (* 0.5 width)) (+ y (* 0.5 height)) (* 0.5 depth)))))]
      (flock. x y width height depth boids))))

(defn heading
  [boid]
  (let
    [velocity (:velocity boid)
     x (get velocity 0)
     y (get velocity 1)]
    (to-degrees (Math/atan2 y x))))

(defn near-it?
  [boid1 boid2 distance key]
  (< (Math/abs (- (key boid1) (key boid2))) distance)
  )

(defn near?
  [b1 b2 d]
  (letfn
    [(f? [boid1 boid2 distance key]
       (< (Math/abs (- (key boid1) (key boid2))) distance))]
    (and
      (f? b1 b2 d :x)
      (f? b1 b2 d :y)
      (f? b1 b2 d :z))))

(defn neighbours
  [boid boids]
  (filter (fn [b] (not (identical? b boid))) boids))

(defn separation-xyz
  [boid boids distance key]
  (let
    [filtered (filter (fn [b] (< (Math/abs (- (key boid) (key b))) distance)) boids)]
    (reduce + (map #(- (key boid) (key %)) filtered))))

(defn separation
  [boid boids distance]
  (let
    [s-boids (neighbours boid boids)]
    (map #(separation-xyz boid s-boids distance %) [:x :y :z])))

(defn alignment
  [boid boids distance]
  (let
    [s-boids (filter (fn [b] (near? b boid distance)) (neighbours boid boids))
     vx (reduce + (map #(get (:velocity %) 0) s-boids))
     vy (reduce + (map #(get (:velocity %) 1) s-boids))
     vz (reduce + (map #(get (:velocity %) 2) s-boids))
     n (count s-boids)]
    (cond (> n 0)
      [ (- (double (/ vx n)) (get (:velocity boid) 0)),
        (- (double (/ vy n)) (get (:velocity boid) 1)),
        (- (double (/ vz n)) (get (:velocity boid) 2)) ]
      :else [0 0 0])))

(defn cohesion
  [boid boids distance]
  (let
    [s-boids (filter (fn [b] (near? b boid distance)) (neighbours boid boids))
     vx (reduce + (map #(:x %) s-boids))
     vy (reduce + (map #(:y %) s-boids))
     vz (reduce + (map #(:z %) s-boids))
     n (count s-boids)]
    (cond (> n 0)
      [ (- (double (/ vx n)) (:x boid)),
        (- (double (/ vy n)) (:y boid)),
        (- (double (/ vz n)) (:z boid)) ]
      :else [0 0 0])))

(defn limit
  [velocity speed]
  (let
    [v velocity
     vx (get v 0)
     vy (get v 1)
     vz (get v 2)
     mm (Math/max
          (Math/max
            (Math/abs vx) (Math/abs vy))
          (Math/abs vz))
     m (if (> mm 0) mm 1)
     nvx (if (> (Math/abs vx) speed) (* (double (/ vx m)) speed) vx)
     nvy (if (> (Math/abs vy) speed) (* (double (/ vy m)) speed) vy)
     nvz (if (> (Math/abs vz) speed) (* (double (/ vz m)) speed) vz)]
    [nvx nvy nvz]))

(defn update-boid
  [boid boids p-separation p-cohesion p-alignment p-target p-limit depth]
  (let
    [f 0.1
     m1 (* p-separation f)
     m2 (* p-cohesion f)
     m3 (* p-alignment f)
     m5 (* p-target f)
     [vx1 vy1 vz1] (separation boid boids (:space boid))
     [vx2 vy2 vz2] (cohesion boid boids (:sight boid))
     [vx3 vy3 vz3] (alignment boid boids (:sight boid))
     target (:target boid)
     [vx5 vy5 vz5] (if (not= target nil) [(- (get target 0) (:x boid)) (- (get target 1) (:y boid)) (- (get target 2) (:z boid))] [0 0 0])
     vx (+ (get (:velocity boid) 0) (* m1 vx1) (* m2 vx2) (* m3 vx3) (* m5 vx5))
     vy (+ (get (:velocity boid) 1) (* m1 vy1) (* m2 vy2) (* m3 vy3) (* m5 vy5))
     vz0 (+ (get (:velocity boid) 2) (* m1 vz1) (* m2 vz2) (* m3 vz3) (* m5 vz5))
     vz (if (> depth 0) vz0 0.0)
     velocity (limit [vx vy vz] p-limit)
     x (+ (:x boid) (get velocity 0))
     y (+ (:y boid) (get velocity 1))
     z (+ (:z boid) (get velocity 2))]
    (assoc boid :x x :y y :z z :velocity velocity)))

(defn constrain-boid
  [boid flock force]
  (let
    [f 5
     bvx (get (:velocity boid) 0)
     bvy (get (:velocity boid) 1)
     bvz (get (:velocity boid) 2)
     vx0 (if (< (:x boid) (:x flock)) (+ bvx (* force f (rand))) bvx)
     vx1 (if (> (:x boid) (+ (:x flock) (:width flock))) (- vx0 (* force f (rand))) vx0)
     vy0 (if (< (:y boid) (:y flock)) (+ bvy (* force f (rand))) bvy)
     vy1 (if (> (:y boid) (+ (:y flock) (:height flock))) (- vy0 (* force f (rand))) vy0)
     vz0 (if (< (:z boid) 0) (+ bvz (* force f (rand))) bvz)
     vz1 (if (> (:z boid) (:depth flock)) (- vz0 (* force f (rand))) vz0)]
    (assoc boid :velocity [vx1 vy1 vz1])))

(defn constrain-flock
  ([flock] (constrain-flock 1.0))
  ([flock force]
    (let
      [boids (map #(constrain-boid % flock force) (:boids flock))]
      (assoc flock :boids boids))))

(defn update-flock
  [flock separation cohesion alignment target limit constrain]
  (let
    [ boids-prev (:boids flock)
      boids (map #(update-boid % boids-prev separation cohesion alignment target limit (:depth flock)) boids-prev)
      updated-flock (assoc flock :boids boids)]
    (constrain-flock updated-flock constrain)))

(defn get-boids
  [flock]
  (:boids flock))

(defn get-boid-xyz
  [boid]
  [(:x boid) (:y boid) (:z boid)])

(defn get-boid-depth
  [boid flock]
  (if (= (:depth flock) 0) 1.0 (Math/max (Math/min (double (/ (:z boid) (:depth flock))) 1.0) 0.0)))
