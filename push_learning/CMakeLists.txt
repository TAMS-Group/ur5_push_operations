cmake_minimum_required(VERSION 2.8.3)
project(push_learning)

find_package(catkin REQUIRED COMPONENTS
	rospy
	ur5_push_setup
	visualization_msgs
	tams_ur5_push_execution)

include_directories(
)

catkin_package(CATKIN_DEPENDS)

catkin_install_python(PROGRAMS scripts/push_result_handler.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
