cmake_minimum_required(VERSION 2.8.3)
project(push_prediction)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	roslib
	geometry_msgs
	tf
	tams_ur5_push_execution
	eigen_conversions)

find_package(Eigen3 REQUIRED)


catkin_package(
	INCLUDE_DIRS
	include

	CATKIN_DEPENDS
	roscpp
	eigen_conversions

	DEPENDS
	EIGEN3
)

include_directories(include
	include/push_prediction
	${catkin_INCLUDE_DIRS}
	${Eigen3_INCLUDE_DIRS}
)


add_executable(push_predictor_test src/push_predictor.cpp)
target_link_libraries(push_predictor_test ${catkin_LIBRARIES} yaml-cpp)
add_dependencies(push_predictor_test ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_library(push_predictor src/push_predictor.cpp)
target_link_libraries(push_predictor ${catkin_LIBRARIES} yaml-cpp)
add_dependencies(push_predictor ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
