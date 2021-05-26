( define ( problem problem_1 )
( :domain clean_up )

(:objects 
	foodtray - zone
	living_room - zone
	foodTray_sz_1 - placePoint 
	foodtray_sz_2 - placePoint
	foodtray_sz_3 - placePoint
	foodtray_sz_4 - placePoint
	foodtray_sz_5 - placePoint
	foodtray_sz_6 - placePoint
    r2d2 - robot    
    lemon - object
)

( :init
	( placePoint_at foodTray_sz_1 foodTray )
	( placePoint_at foodTray_sz_2 foodTray )
	( placePoint_at foodTray_sz_3 foodTray )
	( placePoint_at foodTray_sz_4 foodTray )
	( placePoint_at foodTray_sz_5 foodTray )
	( placePoint_at foodTray_sz_6 foodTray )
	( free foodTray_sz_1 )
	( free foodTray_sz_2 )
	( free foodTray_sz_3 )
	( free foodTray_sz_4 )
	( free foodTray_sz_5 )
	( free foodTray_sz_6 )

	( robot_at r2d2 living_room )
	( object_picked r2d2 lemon )
	(= ( placePoint_at_weigth foodTray_sz_1 foodTray) 0.6)
	(= ( placePoint_at_weigth foodTray_sz_2 foodTray) 0.2)
	(= ( placePoint_at_weigth foodTray_sz_3 foodTray) 0.3)
	(= ( placePoint_at_weigth foodTray_sz_4 foodTray) 0.4)
	(= ( placePoint_at_weigth foodTray_sz_5 foodTray) 0.5)
	(= ( placePoint_at_weigth foodTray_sz_6 foodTray) 0.7)
	(= ( weigth ) 0.0)

)
( :goal
	(and
		(object_in lemon foodTray)
	)	
)
(:metric minimize (weigth))
)
