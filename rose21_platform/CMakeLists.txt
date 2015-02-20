cmake_minimum_required (VERSION 2.4.6)
project(rose21_platform)

find_package(catkin REQUIRED COMPONENTS 
	contact_sensor_msgs
	rose_footprint_collision_checker
	geometry_msgs
	operator_messaging
	opteq_wheelunits_01
	roscomm
	roscpp
	rose_base_msgs
	rose_conversions
	rose_geometry
	rose_transformations
	rose_common
	sensor_msgs
	server_multiple_client
	rose_shared_variables
)

catkin_package(
    INCLUDE_DIRS 
    LIBRARIES 
	CATKIN_DEPENDS 
		contact_sensor_msgs
		rose_footprint_collision_checker
		geometry_msgs
		operator_messaging
		opteq_wheelunits_01
		roscomm
		roscpp
		rose_base_msgs
		rose_conversions
		rose_geometry
		rose_transformations
		rose_common
		sensor_msgs
		server_multiple_client
		rose_shared_variables
	DEPENDS 		 
)
