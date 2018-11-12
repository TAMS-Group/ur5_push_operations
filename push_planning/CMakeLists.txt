cmake_minimum_required(VERSION 2.8.3)
project(tams_ur5_push_planning)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    graph_msgs
    tf
    actionlib
    tams_ur5_push_msgs
    tams_ur5_push_execution
    geometry_msgs
    eigen_conversions
    push_prediction
    moveit_msgs
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    roscpp)

## System dependencies are found with CMake's conventions
find_package(ompl REQUIRED)

catkin_package(
    LIBRARIES
    ${OMPL_LIBRARIES}
    ${OMPL_INCLUDE_DIRS}
    )

include_directories(
    ${EIGEN__INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    include
    )

add_executable(push_planner_node src/push_planner_node.cpp)
add_dependencies(push_planner_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(push_planner_node ${catkin_LIBRARIES} ${OMPL_LIBRARIES} push_sampler push_predictor)
