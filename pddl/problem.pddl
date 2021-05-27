( define ( problem problem_1 )
	( :domain clean_up )

	(:objects 
		r2d2 - robot
		lemon - object
		spam - object
		soup - object
		cloth - object
		bleach - object
		banana - object
	)

	( :init
		(is_free r2d2)
	)
	( :goal
		(and
			(is_placed lemon)
			(is_placed spam)
			(is_placed soup)
			(is_placed cloth)
			(is_placed bleach)
			(is_placed banana)						
		)	
	)
)