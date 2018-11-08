cmake_minimum_required(VERSION 2.8.3)
project(tams_ur5_pushing_object_localization)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
    apriltags2_ros
    roscpp
    visualization_msgs
    tf
    sensor_msgs
    tams_ur5_push_msgs
    image_transport
    cv_bridge
)

include_directories(
# include
 ${catkin_INCLUDE_DIRS}
)

add_executable(image_dump_service src/image_dump_service.cpp)
target_link_libraries(image_dump_service ${catkin_LIBRARIES})
add_dependencies(image_dump_service ${catkin_EXPORTED_TARGETS})

add_executable(object_localization src/object_localization.cpp)
target_link_libraries(object_localization ${catkin_LIBRARIES})
add_dependencies(object_localization ${catkin_EXPORTED_TARGETS})
