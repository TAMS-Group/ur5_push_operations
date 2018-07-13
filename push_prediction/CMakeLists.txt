cmake_minimum_required(VERSION 2.8.3)
project(push_prediction)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	eigen_conversions)

find_package(Eigen3 REQUIRED)

include_directories(include
	include/push_prediction
	${catkin_INCLUDE_DIRS}
	${Eigen3_INCLUDE_DIRS}
)

catkin_package(
	INCLUDE_DIRS

	CATKIN_DEPENDS
	roscpp
	eigen_conversions

	DEPENDS
	EIGEN3

	INCLUDE_DIRS
	include
)

add_library(push_prediction src/push_prediction.cpp)
add_dependencies(push_prediction ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(push_prediction ${catkin_LIBRARIES} yaml-cpp)
