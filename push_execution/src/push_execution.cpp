#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <tf/transform_listener.h>

#include <eigen_conversions/eigen_msg.h>

#include <visualization_msgs/Marker.h>

#include <ur5_pusher/pusher.h>
#include <ur5_pusher/push_approach_sampler.h>

#include <tams_ur5_push_execution/Push.h>
#include <tams_ur5_push_execution/PushApproach.h>

std::string MARKER_TOPIC = "/pushable_objects";

const float MIN_TABLE_DISTANCE = 0.02;
const float WORKABLE_TIP_LENGTH = 0.08;

// Range to restrict the object on the table
const float SAFETY_RANGE = 0.15; // Outside of this range the object is pushed towards the center
const float EMERGENCY_RANGE = 0.3; // Outside of this range the experiment is aborted


namespace tams_ur5_push_execution
{
	class PushExecution
	{
		private:

			ros::NodeHandle nh_;
			ros::NodeHandle pnh_;

			ros::Subscriber marker_sub_;
			ros::Publisher contact_point_pub_;

			moveit::planning_interface::PlanningSceneInterface psi_;
			planning_scene::PlanningSceneConstPtr scene_;

			ur5_pusher::Pusher& pusher_;
			ur5_pusher::PushApproachSampler push_sampler_;

			tf::TransformListener tf_listener_;

			moveit_msgs::CollisionObject obj_;

			visualization_msgs::Marker marker_;
			ros::Time marker_stamp_;

			const bool execute_plan_;

			bool first_attempt_ = true;

		public:
			PushExecution(ur5_pusher::Pusher& pusher, bool execute_plan=false) : pusher_(pusher), execute_plan_(execute_plan), push_sampler_(SAFETY_RANGE, EMERGENCY_RANGE, MIN_TABLE_DISTANCE, WORKABLE_TIP_LENGTH){

				nh_ = (*new ros::NodeHandle());
				pnh_ = (*new ros::NodeHandle("~"));
				psi_ = (*new moveit::planning_interface::PlanningSceneInterface());
				scene_ = std::make_shared<planning_scene::PlanningScene>(pusher_.getRobotModel());

				pusher_.setPlanningTime(5.0);
				pusher_.setPlannerId("RRTConnectkConfigDefault");

				push_sampler_.setReferenceFrame("/table_top");

				marker_sub_ = nh_.subscribe(MARKER_TOPIC, 1, &PushExecution::onDetectObjects, this);
				contact_point_pub_ = nh_.advertise<visualization_msgs::Marker>("/push_approach", 0);
				marker_.id = -1;
			}

			void performRandomPush() {
				if(!marker_.header.frame_id.empty()) {
					if(ros::Time(0) - marker_stamp_ > ros::Duration(0.5)) {
						ROS_WARN_THROTTLE(10, "Marker not up to date, skipping push");
						return;
					}
					// create push message
					Push push;
					createRandomPushMsg(push);

					//remove collision object in case the last attempt failed
					std::vector<std::string> object_ids;
					object_ids.push_back(obj_.id);
					psi_.removeCollisionObjects(object_ids);

					// declare plan and start state
					moveit::planning_interface::MoveGroupInterface::Plan push_plan;
					robot_state::RobotState start_state(*pusher_.getCurrentState());

					//compute push and retreat trajectory together with start state
					if(computeCartesianPushTraj(push, push_plan.trajectory_, start_state)) {

						// move to pre_push pose on first attempt
						if(execute_plan_) {
							// apply collision object before moving to pre_push
							psi_.applyCollisionObject(obj_);
							pusher_.setJointValueTarget(start_state);
							pusher_.move();
							psi_.removeCollisionObjects(object_ids);
						}

						if(execute_plan_)
							pusher_.execute(push_plan);

						// add collision object after run
						psi_.applyCollisionObject(obj_);

					} else {
						ROS_INFO_STREAM("Failed to plan and execute push trajectory!");
					}
				}
			}

		private:

			void createRandomPushMsg(Push& push) {
				push.mode = Push::LINEAR;
				push_sampler_.setMarker(marker_);
				push.approach = push_sampler_.sampleRandomPushApproach();
				visualizePushApproach(push.approach);
				push.distance = 0.05;
			}


			void visualizePushApproach(const PushApproach& approach) {
				geometry_msgs::Pose pose;
				pose.position = approach.point;
				pose.orientation = approach.normal;
				visualizePushApproach(approach.frame_id, pose, approach.angle);
			}


			void visualizePushApproach(std::string frame_id, geometry_msgs::Pose pose, double angle, int id=0) {
				visualization_msgs::Marker approach;
				approach.type = visualization_msgs::Marker::ARROW;
				approach.header.frame_id = frame_id;
				approach.header.stamp = ros::Time();
				approach.id = id;
				approach.action = visualization_msgs::Marker::ADD;

				geometry_msgs::Pose push_direction;
				push_direction.orientation = tf::createQuaternionMsgFromYaw(angle);

				float arrow_len = 0.04;
				geometry_msgs::Pose arrow_offset;
				arrow_offset.orientation.w = 1;
				arrow_offset.position.x = -arrow_len;

				Eigen::Affine3d offset_affine;
				Eigen::Affine3d direction_affine;
				Eigen::Affine3d pose_affine;
				tf::poseMsgToEigen(arrow_offset, offset_affine);
				tf::poseMsgToEigen(push_direction, direction_affine);
				tf::poseMsgToEigen(pose, pose_affine);
				tf::poseEigenToMsg(pose_affine * direction_affine * offset_affine, approach.pose);

				approach.scale.x = arrow_len;
				approach.scale.y = 0.01;
				approach.scale.z = 0.01;
				approach.color.a = 1.0;
				approach.color.r = 1.0;
				approach.lifetime = ros::Duration(5);
				contact_point_pub_.publish(approach);
			}

			bool computeCartesianPushTraj(tams_ur5_push_execution::Push& push, moveit_msgs::RobotTrajectory& trajectory, robot_state::RobotState& state) {
				if(push.mode == tams_ur5_push_execution::Push::LINEAR) {

					// object pose
					geometry_msgs::PoseStamped obj_pose;
					obj_pose.header.frame_id = push.approach.frame_id;
					obj_pose.pose.orientation.w = 1.0;
					tf_listener_.transformPose("table_top", obj_pose, obj_pose);
					Eigen::Affine3d obj_pose_affine;
					tf::poseMsgToEigen(obj_pose.pose, obj_pose_affine);

					// push direction
					Eigen::Affine3d direction(Eigen::AngleAxis<double>(push.approach.angle, Eigen::Vector3d::UnitZ()));

					// approach point
					Eigen::Affine3d approach_affine(Eigen::Affine3d::Identity());
					geometry_msgs::Pose pose;
					pose.position = push.approach.point;
					pose.orientation = push.approach.normal;
					tf::poseMsgToEigen(pose, approach_affine);
					approach_affine = obj_pose_affine * approach_affine * direction;

					//trajectory distances
					float approach_distance = 0.05;
					float retreat_height = marker_.scale.z + 0.1;

					// fill waypoints
					std::vector<geometry_msgs::Pose> waypoints;
					geometry_msgs::Pose waypoint;
					geometry_msgs::Quaternion orientation;
					orientation.w = 1.0;

					Eigen::Affine3d wp(Eigen::Affine3d::Identity()); // instantiate as identity transform!!!
					wp.translate(Eigen::Vector3d(-approach_distance, 0.0, 0.0));
					tf::poseEigenToMsg(approach_affine * wp, waypoint);
					waypoint.orientation = orientation;
					waypoints.push_back(waypoint);

					wp.translate(Eigen::Vector3d(approach_distance + push.distance, 0.0, 0.0));
					tf::poseEigenToMsg(approach_affine * wp, waypoint);
					waypoint.orientation = orientation;
					waypoints.push_back(waypoint);

					wp.translate(Eigen::Vector3d(-push.distance, 0.0, retreat_height));
					tf::poseEigenToMsg(approach_affine * wp, waypoint);
					waypoint.orientation = orientation;
					waypoints.push_back(waypoint);

					//compute cartesian path
					pusher_.setPoseReferenceFrame("table_top");
					pusher_.setStartStateToCurrentState();
					double success_fraction = pusher_.computeCartesianPushPath(waypoints, 0.05, 3, trajectory);
					if(success_fraction == 1.0) {
						trajectory_processing::IterativeParabolicTimeParameterization iptp;
						robot_trajectory::RobotTrajectory traj(pusher_.getRobotModel(), pusher_.getName());
						traj.setRobotTrajectoryMsg(state, trajectory);
						iptp.computeTimeStamps(traj, 0.2, 0.5);
						traj.getRobotTrajectoryMsg(trajectory);
						return true;
					} else {
						ROS_ERROR_STREAM("Could not plan cartesian push path. Achieved " << success_fraction * 100 << "%");
					}
				}
				return false;
			}

			void onDetectObjects(visualization_msgs::Marker marker) {
				if(marker_.id != marker.id && createCollisionObject(marker)) {
					marker_ = marker;
				}
				marker_stamp_ = marker.header.stamp;
			}

			bool createCollisionObject(visualization_msgs::Marker& marker) {
				if(marker.type == visualization_msgs::Marker::CUBE) {
					obj_.id = marker.header.frame_id + "_collision";
					obj_.header.frame_id = marker.header.frame_id;
					obj_.primitive_poses.resize(1);
					obj_.primitive_poses[0].orientation.w = 1;
					obj_.primitives.resize(1);
					obj_.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
					obj_.primitives[0].dimensions.resize(3);
					obj_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = marker.scale.x;
					obj_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = marker.scale.y;
					obj_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = marker.scale.z;
					return true;
				}
				return false;
			}
	};
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "push_execution_node");
	ros::AsyncSpinner spinner(4);
	spinner.start();
	bool execute = false;
	if(argc > 1) {
		if(argc == 2 && std::strcmp(argv[1], "execute") == 0) {
			ROS_WARN_STREAM("Execution of push operations enabled!");
			execute = true;
		} else
			ROS_WARN_STREAM("Could not process arguments! Pass 'execute' to enable execution of push movements.");
	}
	//moveit::planning_interface::MoveGroupInterface arm("arm");
	ur5_pusher::Pusher arm("arm");
	if(!arm.loadFromAttachedObject()) {
		return 1;
	}

	tams_ur5_push_execution::PushExecution pusher(arm, execute);
	while(ros::ok()) {
		pusher.performRandomPush();
		ros::Duration(3).sleep();
	}
	return 0;
}


