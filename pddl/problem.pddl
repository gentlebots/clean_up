( define ( problem problem_1 )
	( :domain clean_up )

	(:objects 
		r2d2 - robot
		object1 - object
	)

	( :init
		(is_free r2d2)
	)
	( :goal
		(and
			(is_placed object1)						
		)	
	)
)