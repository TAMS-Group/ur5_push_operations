#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>

#include <tams_ur5_push_execution/Push.h>
#include <tams_ur5_push_execution/PushApproach.h>

std::random_device rd;
std::mt19937 gen{rd()};

namespace tams_ur5_push_execution
{
	class Pusher
	{
		public:

			moveit_msgs::CollisionObject& obj_;

			moveit::planning_interface::MoveGroupInterface& group_;

			Pusher(moveit::planning_interface::MoveGroupInterface& group,
					moveit_msgs::CollisionObject& obj) : obj_(obj), group_(group){};

			void setTargetObject(moveit_msgs::CollisionObject& obj) {
				obj_ = obj;
			}

			void performRandomPush() {
				Push push;
				push.mode = Push::LINEAR;
				push.approach = sampleRandomPushApproach(obj_);
				push.distance = 0.05;
				moveit::planning_interface::MoveGroupInterface::Plan plan;
				robot_state::RobotState rstate(*group_.getCurrentState());
				if(computeCartesianPushTraj(push, plan.trajectory_, rstate)) {
					// move to pre-push position
					group_.setJointValueTarget(rstate);
					group_.move();

					// push object
					group_.execute(plan);

					// move endeffector up
					geometry_msgs::PoseStamped pose = group_.getCurrentPose();
					pose.pose.position.z += 0.2;
					group_.setJointValueTarget(pose);
					group_.move();
				} else {
					ROS_INFO_STREAM("Failed to plan and execute push trajectory!");
				}
			}

		private:

			PushApproach sampleRandomPushApproach(moveit_msgs::CollisionObject& obj) {
				PushApproach approach;
				approach.frame_id = obj.header.frame_id;

				geometry_msgs::Pose approach_pose;
				sampleRandomContactPoint(obj, approach_pose);

				approach.point = approach_pose.position;
				approach.normal = approach_pose.orientation;
				approach.angle = sampleRandomPushAngle();
				return approach;
			}

			bool computeCartesianPushTraj(tams_ur5_push_execution::Push& push, moveit_msgs::RobotTrajectory& trajectory, robot_state::RobotState& rstate) {
				if(push.mode == tams_ur5_push_execution::Push::LINEAR) {
					// extract push direction and point
					tf::Quaternion push_orientation;
					tf::quaternionMsgToTF(tf::createQuaternionMsgFromYaw(push.approach.angle), push_orientation);
					tf::Quaternion push_normal;
					tf::quaternionMsgToTF(push.approach.normal, push_normal);
					tf::Quaternion push_direction = push_normal + push_orientation;
					tf::Vector3 approach_point;
					tf::pointMsgToTF(push.approach.point, approach_point);

					tf::Transform approach;
					approach.setOrigin(approach_point);
					approach.setRotation(push_direction);

					tf::Transform start_point;
					geometry_msgs::Point start;
					start.x = -0.1; //TODO: move pre-approach distance to msg
					tf::Vector3 start_v;
					tf::pointMsgToTF(start,start_v);
					start_point.setOrigin(start_v);
					start_point.setRotation(tf::Quaternion::getIdentity());
					start_point *= approach;

					tf::Transform goal_point;
					geometry_msgs::Point goal;
					start.x = push.distance;
					tf::Vector3 goal_v;
					tf::pointMsgToTF(goal,goal_v);
					goal_point.setOrigin(goal_v);
					goal_point.setRotation(tf::Quaternion::getIdentity());
					goal_point *= approach;

					std::vector<geometry_msgs::Pose> waypoints;
					int step_count = 10;
					tf::Vector3 step = (goal_point.getOrigin() - start_point.getOrigin()) / step_count;
					tf::Vector3 start_origin = start_point.getOrigin();
					geometry_msgs::Pose wp;
					wp.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.5*M_PI, 0.0);

					for(int i = 0; i <= step_count; i++) {
						start_point.setOrigin(start_origin + i * step);
						tf::poseTFToMsg(start_point, wp);
						waypoints.push_back(wp);
					}

					group_.setPoseReferenceFrame(push.approach.frame_id);
					group_.setJointValueTarget(waypoints[0]);
					rstate = group_.getJointValueTarget();
					group_.setStartState(rstate);
					float success_fraction = group_.computeCartesianPath(waypoints, 0.03, 3, trajectory);
					group_.setStartStateToCurrentState();
					group_.clearPoseTargets();
					if(success_fraction == 1.0) {
						return true;
					}
				}
				return false;
			}

			bool sampleRandomContactPoint(moveit_msgs::CollisionObject& obj, geometry_msgs::Pose& pose) {
				// we expect a single BOX primitive for now
				geometry_msgs::Point contact_point;
				//Sample contact point from obj shape
				if(obj.primitives[0].type == shape_msgs::SolidPrimitive::BOX) {
					double dim_x = obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X];
					double dim_y = obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y];
					pose.position.z = obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z];
					double box_len = 2 * (dim_x + dim_y);
					std::uniform_real_distribution<> dis(0.0, box_len);
					double p = dis(gen);
					if(p < dim_x) {
						pose.position.x = p;
						pose.position.y = 0;
						pose.orientation.w = 1;
					} else if (p < dim_x + dim_y) {
						pose.position.x = dim_x;
						pose.position.y = p - dim_y;
						pose.orientation = tf::createQuaternionMsgFromYaw(0.5*M_PI);
					} else if (p < 2 * dim_x + dim_y) {
						pose.position.x = 2 * dim_x + dim_y - p;
						pose.position.y = dim_y;
						pose.orientation = tf::createQuaternionMsgFromYaw(M_PI);
					} else {
						pose.position.x = 0;
						pose.position.y = 2 * (dim_x + dim_y) - p;
						pose.orientation = tf::createQuaternionMsgFromYaw(1.5*M_PI);
					}
					// adjust to center
					pose.position.x -= 0.5*dim_x;
					pose.position.y -= 0.5*dim_y;
					return true;
				}
				return false;
			}

			float sampleRandomPushAngle() {
				std::normal_distribution<> d{0,0.2};
				return d(gen);
			}
	};
}

moveit_msgs::CollisionObject spawnObject(moveit::planning_interface::PlanningSceneInterface& psi, const std::string& obj_id, double x_dim, double y_dim, double z_dim, double x_pos=0.0, double y_pos=0.0){
	moveit_msgs::CollisionObject obj;
	obj.id = obj_id;
	obj.header.frame_id = "table_top";
	obj.primitive_poses.resize(1);
	obj.primitive_poses[0].position.x = x_pos;
	obj.primitive_poses[0].position.y = y_pos;
	obj.primitive_poses[0].position.z = 0.5 * z_dim;
	obj.primitive_poses[0].orientation.w = 1;
	obj.primitives.resize(1);
	obj.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
	obj.primitives[0].dimensions.resize(3);
	obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = x_dim;
	obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = y_dim;
	obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = z_dim;
	//psi.applyCollisionObject(obj);
	return obj;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "push_execution_node");
	ros::AsyncSpinner spinner(4);
	spinner.start();
	moveit::planning_interface::PlanningSceneInterface psi;
	moveit_msgs::CollisionObject box = spawnObject(psi, "box", 0.05, 0.1, 0.05); 
	moveit::planning_interface::MoveGroupInterface arm("arm");
	tams_ur5_push_execution::Pusher pusher(arm, box);
	pusher.performRandomPush();
	return 0;
}


