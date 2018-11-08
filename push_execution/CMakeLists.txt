cmake_minimum_required(VERSION 2.8.3)
project(tams_ur5_push_execution)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  roscpp
  geometry_msgs
  shape_msgs
  std_msgs
  moveit_msgs
  control_msgs
  visualization_msgs
  object_recognition
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  tf
  tf_conversions
  eigen_conversions
  message_generation)

add_action_files(
  DIRECTORY action
  FILES ExplorePushes.action MoveObject.action
  )

add_message_files(
  FILES
  PushApproach.msg
  Push.msg
  PushTrajectory.msg
  )

add_service_files(
  FILES
  PerformRandomPush.srv
  ExecutePush.srv
  PusherMovement.srv
  SamplePredictivePush.srv
  PredictPush.srv
  SteerPush.srv
  )

generate_messages(
  DEPENDENCIES
  geometry_msgs
  actionlib_msgs
  )

catkin_package(INCLUDE_DIRS
  include
  CATKIN_DEPENDS
  message_runtime)

#set(PROJECT_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/include)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

add_library(pusher src/push_execution/pusher.cpp)

add_library(push_sampler src/push_sampler/exploration_sampler.cpp src/push_sampler/push_sampler.cpp)
add_dependencies(push_sampler ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(push_execution src/push_execution/push_execution_server.cpp)
target_link_libraries(push_execution ${catkin_LIBRARIES} pusher push_sampler)
add_dependencies(push_execution ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ library
add_executable(cli src/cli.cpp)
target_link_libraries(cli ${catkin_LIBRARIES} pusher)
add_dependencies(cli ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(DIRECTORY include DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
