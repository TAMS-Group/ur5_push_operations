cmake_minimum_required(VERSION 2.8.3)
project(object_recognition)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
    apriltags_ros
    roscpp
    tf
)

catkin_package(
)

include_directories(
# include
 ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAMWE} ${catkin_EXPORTED_TARGETS})
