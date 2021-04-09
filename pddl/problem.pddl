( define ( problem problem_1 )
( :domain clean_up )
( :objects
	sugar_box lemon - object
	foodTray outdoor living_room - zone
	foodTray_sz_1 foodTray_sz_2 foodTray_sz_3 foodTray_sz_4 foodTray_sz_5 foodTray_sz_6 - subzone
	r2d2 - robot
)
( :init
	( subzone_at foodTray_sz_1 foodTray )
	( subzone_at foodTray_sz_2 foodTray )
	( subzone_at foodTray_sz_3 foodTray )
	( subzone_at foodTray_sz_4 foodTray )
	( subzone_at foodTray_sz_5 foodTray )
	( subzone_at foodTray_sz_6 foodTray )
	( free foodTray_sz_1 )
	( free foodTray_sz_2 )
	( free foodTray_sz_3 )
	( free foodTray_sz_4 )
	( free foodTray_sz_5 )
	( free foodTray_sz_6 )

	( robot_at r2d2 living_room )
	( object_picked r2d2 lemon )
	(= ( subzone_at_weigth foodTray_sz_1 foodTray) 0.6)
	(= ( subzone_at_weigth foodTray_sz_2 foodTray) 0.2)
	(= ( subzone_at_weigth foodTray_sz_3 foodTray) 0.3)
	(= ( subzone_at_weigth foodTray_sz_4 foodTray) 0.4)
	(= ( subzone_at_weigth foodTray_sz_5 foodTray) 0.5)
	(= ( subzone_at_weigth foodTray_sz_6 foodTray) 0.7)
	(= ( weigth ) 0.0)

)
( :goal
	( and
		( object_at lemon foodTray)
	)
)
(:metric minimize (weigth))
)
