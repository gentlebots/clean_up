(define (domain clean_up)
(:requirements :strips :typing :adl :fluents :durative-actions :typing)

;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:types
object
zone 
robot
subzone
);; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
(:predicates

(robot_at ?r - robot ?z - zone)
(object_at ?o - object ?z - zone)

(subzone_at ?sz - subzone ?z - zone)
(free ?sz - subzone)

(is_delivery_zone ?z - zone)
(is_foodTray_zone ?z - zone)

(object_picked ?r - robot ?o - object)

);; end Predicates ;;;;;;;;;;;;;;;;;;;;


;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;
(:functions

(subzone_at_weigth ?sz - subzone ?z - zone)
(weigth)

);; end Functions ;;;;;;;;;;;;;;;;;;;;

;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:durative-action move
    :parameters (?r - robot ?z1 ?z2 - zone)
    :duration ( = ?duration 5)
    :condition (and
        (at start(robot_at ?r ?z1)))
    :effect (and
        (at start(not(robot_at ?r ?z1)))
        (at end(robot_at ?r ?z2))
    )
)

(:durative-action pick
    :parameters (?r - robot ?o - object)
    :duration ( = ?duration 5)
    :condition (and
    )
    :effect (and
        (at end(object_picked ?r ?o))
    )
)

(:durative-action place
    :parameters (?r - robot ?o - object ?z - zone ?sz - subzone)
    :duration ( = ?duration 5)
    :condition (and
        (at start(robot_at ?r ?z))
        (at start(object_picked ?r ?o))
        (at start(subzone_at ?sz ?z))
        (at start(free ?sz))
    )
    :effect (and
        (at end(not(free ?sz)))
        (at end(object_at ?o ?z))
        (at end(increase (weigth) (subzone_at_weigth ?sz ?z)))
    )
)

);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;
