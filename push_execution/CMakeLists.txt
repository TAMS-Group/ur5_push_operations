cmake_minimum_required(VERSION 2.8.3)
project(tams_ur5_push_execution)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	geometry_msgs
	std_msgs
	visualization_msgs
	moveit_msgs
	moveit_core
	moveit_ros_planning
	moveit_ros_planning_interface
	tf
	tf_conversions
	eigen_conversions
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

catkin_package(INCLUDE_DIRS
		include
		CATKIN_DEPENDS 
		message_runtime)

set(PROJECT_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/include/ur5_pusher)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
 ${catkin_INCLUDE_DIRS}
)

add_library(ur5_pusher src/pusher.cpp)
add_library(push_approach_sampler src/push_approach_sampler.cpp)

add_executable(push_execution src/push_execution.cpp)
target_link_libraries(push_execution ${catkin_LIBRARIES} ur5_pusher push_approach_sampler)

## Declare a C++ library
add_executable(pusher src/pusher.cpp)
target_link_libraries(pusher ${catkin_LIBRARIES})

add_executable(push_bringup src/push_bringup.cpp)
target_link_libraries(push_bringup ${catkin_LIBRARIES} ur5_pusher)
#add_dependencies(push_bringup ${${PROJECT_NAME}_EXPORTED_TARGETS})

install(DIRECTORY include DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
