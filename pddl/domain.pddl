(define (domain clean_up)
(:requirements :strips :typing :equality :adl :fluents :durative-actions :typing :quantified-preconditions)

;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:types
    object
    zone
    placePoint
    robot
);; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
(:predicates
	( placePoint_at ?sz - placePoint ?z - zone )
	( free ?sz - placePoint )
	( robot_at ?r - robot  ?sz - zone )
	( object_picked ?r - robot ?o - object )
	( object_at ?o - object ?sz - placePoint )	
	( object_in ?o - object ?z - zone) 	
);; end Predicates ;;;;;;;;;;;;;;;;;;;;


;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;
(:functions
    (placePoint_at_weigth ?sz - placePoint ?z - zone )
    (weigth)
);; end Functions ;;;;;;;;;;;;;;;;;;;;

;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;; From gb_manipulation!
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
    :parameters (?r - robot ?o - object ?z - zone)
    :duration ( = ?duration 5)
    :condition (and
        (at start(not(object_picked ?r ?o)))
        (at start(object_in ?o ?z))
        )
    :effect (and
        (at end(object_picked ?r ?o))
        (at end(not(object_in ?o ?z)))
        )
)

(:durative-action place
    :parameters (?r - robot ?o - object ?pP - placePoint ?z - zone)
    :duration ( = ?duration 5)
    :condition (and
        (at start(object_picked ?r ?o))
        (at start(robot_at ?r ?z))
        (at start(placePoint_at ?pP ?z))
        
    )
    :effect (and
        (at end(not(object_picked ?r ?o)))
        (at end(object_at ?o ?pP))
        (at end(object_in ?o ?z))
        (at end(increase (weigth) (placePoint_at_weigth ?pP ?z)))
    )
)

);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;
