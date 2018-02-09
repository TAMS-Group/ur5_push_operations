#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>

#include <eigen_conversions/eigen_msg.h>

#include <visualization_msgs/Marker.h>

#include <tams_ur5_push_execution/Push.h>
#include <tams_ur5_push_execution/PushApproach.h>

std::random_device rd;
std::mt19937 gen{rd()};

std::string MARKER_TOPIC = "/pushable_objects";

namespace tams_ur5_push_execution
{
	class Pusher
	{
		private:

			ros::NodeHandle nh_;
			ros::NodeHandle pnh_;

			ros::Subscriber marker_sub_;
			ros::Publisher contact_point_pub_;

			visualization_msgs::Marker* marker_ = NULL;

		public:

			moveit_msgs::CollisionObject& obj_;

			moveit::planning_interface::MoveGroupInterface& group_;

			Pusher(moveit::planning_interface::MoveGroupInterface& group,
					moveit_msgs::CollisionObject& obj) : obj_(obj), group_(group){

				nh_ = (*new ros::NodeHandle());
				pnh_ = (*new ros::NodeHandle("~"));

				marker_sub_ = nh_.subscribe(MARKER_TOPIC, 1, &Pusher::onDetectObjects, this);
				contact_point_pub_ = nh_.advertise<visualization_msgs::Marker>("/push_approach", 0);
			};

			void performRandomPush() {
				if(marker_ != NULL) {
                    Push push;
                    push.mode = Push::LINEAR;
                    push.approach = sampleRandomPushApproach();
                    push.distance = 0.05;
                    /*
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
                    */
                }
			}

		private:

			PushApproach sampleRandomPushApproach() {
				PushApproach approach;
				//approach.frame_id = marker_->header.frame_id;
                approach.frame_id = "/pushable_object_0";

				geometry_msgs::Pose approach_pose;
				sampleRandomContactPoint(approach_pose);
				approach.point = approach_pose.position;
				approach.normal = approach_pose.orientation;
				approach.angle = sampleRandomPushAngle();
                // visualize contact point with arrow marker
                visualizePushApproach(approach.frame_id, approach_pose, approach.angle);
				return approach;
			}

            void visualizePushApproach(std::string frame_id, geometry_msgs::Pose pose, double angle) {
                visualization_msgs::Marker approach;
                approach.type = visualization_msgs::Marker::ARROW;
                //approach.header.frame_id = frame_id;
                approach.header.frame_id = "/pushable_object_0";
                approach.header.stamp = ros::Time();
                approach.id = 0;
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

			bool computeCartesianPushTraj(tams_ur5_push_execution::Push& push, moveit_msgs::RobotTrajectory& trajectory, robot_state::RobotState& rstate) {
                if(push.mode == tams_ur5_push_execution::Push::LINEAR) {
                    //push direction
                    geometry_msgs::Pose push_direction;
                    push_direction.orientation = tf::createQuaternionMsgFromYaw(push.approach.angle);

                    //trajectory distances
                    float pre_push_distance = 0.05;
                    float push_distance = 0.05;

                    //pre push offset
                    geometry_msgs::Pose start_pose;
                    start_pose.orientation.w = 1;
                    start_pose.position.x = -pre_push_distance;

                    Eigen::Affine3d start_pose_affine;
                    Eigen::Affine3d direction_affine;
                    Eigen::Affine3d pose_affine;
                    tf::poseMsgToEigen(start_pose, start_pose_affine);
                    tf::poseMsgToEigen(push_direction, direction_affine);
                    geometry_msgs::Pose pose;
                    pose.position = push.approach.point;
                    pose.orientation = push.approach.normal;
                    tf::poseMsgToEigen(pose, pose_affine);
                    start_pose_affine = pose_affine * direction_affine * start_pose_affine;

                    // compute waypoints
					std::vector<geometry_msgs::Pose> waypoints;
					int step_count = 10;
                    float step_size = (pre_push_distance + push_distance) / (float)step_count;
                    geometry_msgs::Pose waypoint;
                    Eigen::Affine3d waypoint_affine;
                    waypoint.orientation.w = 1;
					for(int i = 0; i <= step_count; i++) {
                        waypoint.position.x = i * step_size;
                        tf::poseMsgToEigen(waypoint, waypoint_affine);
                        tf::poseEigenToMsg(start_pose_affine * waypoint_affine, waypoint);
						waypoints.push_back(waypoint);
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

            /**
             * Sample random contact point from marker
             */
			bool sampleRandomContactPoint(geometry_msgs::Pose& pose) {
				// we expect a single BOX primitive for now
				geometry_msgs::Point contact_point;
				//Sample contact point from obj shape
				if(marker_->type == visualization_msgs::Marker::CUBE) {
					double dim_x = marker_->scale.x;
					double dim_y = marker_->scale.y;

					double cube_len = 2 * (dim_x + dim_y);
					std::uniform_real_distribution<> dis(0.0, cube_len);
					double p = dis(gen);
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
                    //TODO: transform pose with marker pose offset
					return true;
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
				std::normal_distribution<> d{0,0.5};
				return d(gen);
			}

			void onDetectObjects(visualization_msgs::Marker marker) {
				marker_ = &marker;
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
    while(ros::ok()) {
        pusher.performRandomPush();
        ros::Duration(3).sleep();
    }
    return 0;
}


