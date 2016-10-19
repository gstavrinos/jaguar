cmake_minimum_required(VERSION 2.8.3)
project(jaguar_description)

find_package(catkin REQUIRED COMPONENTS
	urdf
	xacro
	tf
	roscpp
	sensor_msgs
)

catkin_package()