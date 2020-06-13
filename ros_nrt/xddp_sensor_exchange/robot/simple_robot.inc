define laser ranger(
	sensor(
		range [0.0 5.6]
		fov 240
		samples 240
	)

	color "blue"
	size [0.06 0.06 0.06]

	block(
		points 4
		point[0] [0 0]
		point[1] [0 1]
		point[2] [1 1]
		point[3] [1 0]
		z [0 1]
	)
)

define simple_robot position(
	odom_error [0.03 0.03 0.00 0.05]
	localization "odom"
	gui_nose 1
	drive "diff"
	color "red"

	size [0.35 0.25 0.15]

	
	block(
		points 4
		point[0] [0.25 0]
		point[1] [0.25 0.125]
		point[2] [1.25 0.125]
		point[3] [1.25 0]
		z [0 1]
	)

	block(
		points 4
		point[0] [0.25 0.125]
		point[1] [0.25 0.75]
		point[2] [1 0.75]
		point[3] [0.75 0.125]
		z [0 1]
	)

	block(
		points 4
		point[0] [0.25 0.75]
		point[1] [0.25 1.5]
		point[2] [1.25 1.5]
		point[3] [1.25 0.75]
		z [0 1]
	)

	block(
		points 4
		point[0] [1.25 0.75]
		point[1] [1.25 1.5]
		point[2] [2 1.25 ]
		point[3] [2 1]
		z [0 1]
	)

	block(
		points 4
		point[0] [0 1]
		point[1] [0 1.25]
		point[2] [0.25 1.25]
		point[3] [0.25 1]
		z [0 1]
	)

	block(
		points 4
		point[0] [0.25 1.5]
		point[1] [0.25 2.125]
		point[2] [0.75 2.125]
		point[3] [1 1.5]
		z [0 1]
	)

	block(
		points 4
		point[0] [1.25 0.75]
		point[1] [1.25 1.5]
		point[2] [2 1.25]
		point[3] [2 1]
		z [0 1]
	)

	block(
		points 4
		point[0] [0.25 2.125]
		point[1] [0.25 2.25]
		point[2] [1.25 2.25]
		point[3] [1.25 2.125]
		z [0 1]
	)



	laser(pose [0.205 0.0 -0.1 0.0])
)