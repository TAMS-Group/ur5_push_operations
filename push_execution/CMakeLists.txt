cmake_minimum_required(VERSION 2.8.3)
project(tams_ur5_push_execution)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	moveit_ros_planning_interface
	moveit_core
	moveit_msgs
	geometry_msgs
	eigen_conversions
	visualization_msgs
	tf_conversions
	tf
	message_generation)

add_message_files(
	FILES
	PushApproach.msg
	Push.msg
)

generate_messages(
	DEPENDENCIES
	geometry_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	#include
 ${catkin_INCLUDE_DIRS}
)


## Declare a C++ library
add_executable(pusher src/pusher.cpp)
target_link_libraries(pusher ${catkin_LIBRARIES})


add_executable(push_bringup src/push_bringup.cpp)
target_link_libraries(push_bringup ${catkin_LIBRARIES})
#add_dependencies(push_bringup ${${PROJECT_NAME}_EXPORTED_TARGETS})


