cmake_minimum_required(VERSION 2.8.3)
project(ur5_pushing_object_recognition)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
    apriltags2_ros
    roscpp
    visualization_msgs
    tf
    sensor_msgs
    image_transport
    cv_bridge
    message_generation
)

add_service_files(
	FILES
	ImageDump.srv
)

generate_messages(
	DEPENDENCIES
)

catkin_package(
	CATKIN_DEPENDS message_runtime
)

include_directories(
# include
 ${catkin_INCLUDE_DIRS}
)

add_executable(image_dump_service src/image_dump_service.cpp)
target_link_libraries(image_dump_service ${catkin_LIBRARIES})
add_dependencies(image_dump_service ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME} src/object_recognition.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
