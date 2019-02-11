cmake_minimum_required(VERSION 2.8.3)
project(tams_ur5_push_msgs)

find_package(catkin REQUIRED
	geometry_msgs
	actionlib_msgs
	message_generation
	graph_msgs
	)

add_action_files(
  DIRECTORY action FILES
  ExplorePushes.action
  MoveObject.action
  PlanPush.action
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
  ImageDump.srv
  FTDump.srv
  )

generate_messages(
  DEPENDENCIES
  geometry_msgs
  actionlib_msgs
  graph_msgs
  )


catkin_package(
  CATKIN_DEPENDS
  message_runtime
  )
