cmake_minimum_required(VERSION 2.8.3)
project(push_planning)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    tams_ur5_push_execution
    geometry_msgs
    eigen_conversions
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
    )


add_executable(push_planning_node src/RigidBodyPlanningWithControls.cpp)

target_link_libraries(push_planning_node
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
)
