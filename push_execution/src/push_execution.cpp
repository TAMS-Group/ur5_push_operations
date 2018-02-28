#include <algorithm>

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
#include <tams_ur5_push_execution/Push.h>
#include <tams_ur5_push_execution/PushApproach.h>

std::random_device rd;
std::mt19937 gen{rd()};

std::string MARKER_TOPIC = "/pushable_objects";

float MIN_TABLE_DISTANCE = 0.02;
float WORKABLE_TIP_LENGTH = 0.08;

// Range to restrict the object on the table
float SAFETY_RANGE = 0.15; // Outside of this range the object is pushed towards the center
float EMERGENCY_RANGE = 0.3; // Outside of this range the experiment is aborted


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

			tf::TransformListener tf_listener_;

			moveit_msgs::CollisionObject obj_;

			visualization_msgs::Marker marker_;
			ros::Time marker_stamp_;

			const bool execute_plan_;

			bool first_attempt_ = true;

		public:
			PushExecution(ur5_pusher::Pusher& pusher, bool execute_plan=false) : pusher_(pusher), execute_plan_(execute_plan){

				nh_ = (*new ros::NodeHandle());
				pnh_ = (*new ros::NodeHandle("~"));
				psi_ = (*new moveit::planning_interface::PlanningSceneInterface());
				scene_ = std::make_shared<planning_scene::PlanningScene>(pusher_.getRobotModel());

				pusher_.setPlanningTime(5.0);
				pusher_.setPlannerId("RRTConnectkConfigDefault");

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
						if(execute_plan_ && first_attempt_) {
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
				push.approach = sampleRandomPushApproach();
				push.distance = 0.05;
			}

			PushApproach sampleRandomPushApproach() {
				PushApproach approach;
				approach.frame_id = marker_.header.frame_id;

				geometry_msgs::Pose approach_pose;
				double angle = 0.0;
				sampleApproachPoseAndAngle(marker_, approach_pose, angle);
				approach.angle = angle;

				//geometry_msgs::Pose approach_pose = 
				//    sampleRandomPointFromBox(marker_.scale.x, marker_.scale.y, marker_.scale.z);
				//approach.angle = sampleRandomPushAngle();

				approach.point = approach_pose.position;
				approach.normal = approach_pose.orientation;

				// visualize contact point with arrow marker
				visualizePushApproach(approach.frame_id, approach_pose, approach.angle);
				return approach;
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
					wp.translate(Eigen::Vector3d(-approach_distance, 0.0, retreat_height));
					tf::poseEigenToMsg(approach_affine * wp, waypoint);
					waypoint.orientation = orientation;
					waypoints.push_back(waypoint);

					wp.translate(Eigen::Vector3d(0.0, 0.0, -retreat_height));
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

			bool sampleApproachPoseAndAngle(visualization_msgs::Marker& marker, geometry_msgs::Pose& pose, double& angle, int attempts=100) {
				geometry_msgs::PoseStamped object_pose, marker_pose;
				marker_pose.pose = marker.pose;
				marker_pose.header.frame_id = marker.header.frame_id;
				tf_listener_.transformPose("/table_top", marker_pose, object_pose);
				float distance = std::sqrt(std::pow(object_pose.pose.position.x,2) + std::pow(object_pose.pose.position.y,2));
				if(distance < SAFETY_RANGE) {
					pose = sampleRandomPointFromBox(marker_.scale.x, marker_.scale.y, marker_.scale.z);
					angle = sampleRandomPushAngle();
					return true;
				}
				if(distance < EMERGENCY_RANGE) {
					ROS_WARN("Object outside of SAFETY RANGE. Sampling for pushes towards table center!");
					tf::Vector3 table_vec;
					object_pose.pose.position.z = 0.0;
					tf::pointMsgToTF(object_pose.pose.position, table_vec);
					table_vec = -table_vec;

					tf::Quaternion obj_orientation;
					tf::Quaternion push_normal;
					tf::Quaternion push_direction;
					push_direction.setRPY(0.0, 0.0, angle);
					tf::quaternionMsgToTF(object_pose.pose.orientation, obj_orientation);
					tf::Vector3 push_vec(1,0,0);

					for(int i = 0; i < attempts; i++) {
						pose = sampleRandomPointFromBox(marker_.scale.x, marker_.scale.y, marker_.scale.z);
						angle = sampleRandomPushAngle();
						tf::quaternionMsgToTF(pose.orientation, push_normal);
						tf::Vector3 push_vec(1,0,0);
						push_vec = tf::quatRotate(obj_orientation * push_normal * push_direction, push_vec);
						double angle_limit = 20.0 / 180.0 * M_PI;
						double angle_towards_table = table_vec.angle(push_vec);
						if(angle_towards_table < angle_limit || angle_towards_table > (2*M_PI - angle_limit)) {
							return true;
						}
					}
					ROS_ERROR_STREAM("Push experiment aborted! Could not sample valid push pose in " << attempts << " attempts!");
				}  else {
					ROS_ERROR_STREAM("Push experiment aborted! Object is outside of safety range.");
				}
				return false;
			}

			/**
			 * Sample random contact point from box dimensions
			 */
			geometry_msgs::Pose sampleRandomPointFromBox(double dim_x, double dim_y, double dim_z) {
				geometry_msgs::Pose pose;
				// Pick random value in range of perimeter
				std::uniform_real_distribution<> dis(0.0, 2 * (dim_x + dim_y));
				double p = dis(gen);
				// Match value with edge and create corresponding pose
				if(p <= dim_x) {
					pose.position.x = p;
					pose.position.y = 0;
					pose.orientation = tf::createQuaternionMsgFromYaw(0.5*M_PI);
				} else if (p <= dim_x + dim_y) {
					pose.position.x = dim_x;
					pose.position.y = p - dim_x;
					pose.orientation = tf::createQuaternionMsgFromYaw(M_PI);
				} else if (p <= 2 * dim_x + dim_y) {
					pose.position.x = 2 * dim_x + dim_y - p;
					pose.position.y = dim_y;
					pose.orientation = tf::createQuaternionMsgFromYaw(1.5*M_PI);
				} else {
					pose.position.x = 0;
					pose.position.y = 2 * (dim_x + dim_y) - p;
					pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
				}
				// adjust to center
				pose.position.x -= 0.5*dim_x;
				pose.position.y -= 0.5*dim_y;

				// Pose height is related to box height and tip length
				// By default the tip aligns with the frame of the box.
				// The tip must be lifted in two mutually exclusive cases:
				// 1. The box is too small and therefore the table distance too short.
				pose.position.z = std::max(MIN_TABLE_DISTANCE - 0.5 * dim_z, 0.0);
				// 2. The box is too high for the tip and might touch the gripper
				pose.position.z = std::max(0.5 * dim_z - WORKABLE_TIP_LENGTH, pose.position.z);
				return pose;
			}

			float sampleRandomPushAngle(int degrees=30) {
				float range = M_PI * degrees / 180.0;
				std::normal_distribution<> d{0,0.5};
				return std::max(-range, std::min(range, (float)d(gen)));
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


