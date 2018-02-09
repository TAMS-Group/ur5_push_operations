#include <algorithm>

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/CollisionObject.h>

#include <tf/transform_listener.h>

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

            moveit::planning_interface::PlanningSceneInterface psi_;
            planning_scene::PlanningSceneConstPtr scene_;

            moveit::planning_interface::MoveGroupInterface& group_;

            tf::TransformListener tf_listener_;

            moveit_msgs::CollisionObject obj_;

            visualization_msgs::Marker* marker_ = NULL;

        public:
            Pusher(moveit::planning_interface::MoveGroupInterface& group) : group_(group){

                nh_ = (*new ros::NodeHandle());
                pnh_ = (*new ros::NodeHandle("~"));
                psi_ = (*new moveit::planning_interface::PlanningSceneInterface());
                scene_ = std::make_shared<planning_scene::PlanningScene>(group_.getRobotModel());

                group_.setPlanningTime(5.0);
                group_.setPlannerId("RRTConnectkConfigDefault");

                marker_sub_ = nh_.subscribe(MARKER_TOPIC, 1, &Pusher::onDetectObjects, this);
                contact_point_pub_ = nh_.advertise<visualization_msgs::Marker>("/push_approach", 0);
            };

            void performRandomPush() {
                if(marker_ != NULL) {
                    // create push message
                    Push push;
                    createRandomPushMsg(push);

                    //remove collision object in case the last attempt failed
                    std::vector<std::string> object_ids;
                    object_ids.push_back(obj_.id);
                    psi_.removeCollisionObjects(object_ids);

                    // declare plan and start state
                    moveit::planning_interface::MoveGroupInterface::Plan push_plan;
                    robot_state::RobotState start_state(*group_.getCurrentState());

                    //compute push and retreat trajectory together with start state
                    if(computeCartesianPushTraj(push, push_plan.trajectory_, start_state)) {

                        // apply collision object before moving to pre_push
                        psi_.applyCollisionObject(obj_);

                        // move to pre_push pose
                        group_.setJointValueTarget(start_state);
                        group_.move();

                        // remove collision object before push and for next plan
                        psi_.removeCollisionObjects(object_ids);

                        // push object and retreat
                        group_.execute(push_plan);

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

            void visualizePushApproach(std::string frame_id, geometry_msgs::Pose pose, double angle, int id=0) {
                visualization_msgs::Marker approach;
                approach.type = visualization_msgs::Marker::ARROW;
                approach.header.frame_id = frame_id;
                //approach.header.frame_id = "/pushable_object_0";
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
                    // contact point
                    geometry_msgs::Pose pose;
                    pose.position = push.approach.point;
                    pose.orientation = push.approach.normal;

                    //push direction
                    geometry_msgs::Pose push_direction;
                    push_direction.orientation = tf::createQuaternionMsgFromYaw(push.approach.angle);

                    //trajectory distances
                    float pre_push_distance = 0.05;

                    geometry_msgs::Quaternion pusher_orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.5*M_PI, 0.0);

                    // start pose
                    geometry_msgs::Pose start_pose;
                    start_pose.orientation = pusher_orientation;
                    start_pose.position.x = -pre_push_distance;

                    // goal pose
                    geometry_msgs::Pose goal_pose;
                    goal_pose.orientation = pusher_orientation;
                    goal_pose.position.x = push.distance;

                    // retreat pose
                    geometry_msgs::Pose retreat_pose;
                    retreat_pose.orientation = pusher_orientation;
                    retreat_pose.position.x = push.distance * 0.8;
                    retreat_pose.position.z += 0.1;

                    // create affine transforms
                    Eigen::Affine3d start_pose_affine;
                    Eigen::Affine3d goal_pose_affine;
                    Eigen::Affine3d retreat_pose_affine;
                    Eigen::Affine3d direction_affine;
                    Eigen::Affine3d pose_affine;
                    tf::poseMsgToEigen(start_pose, start_pose_affine);
                    tf::poseMsgToEigen(goal_pose, goal_pose_affine);
                    tf::poseMsgToEigen(retreat_pose, retreat_pose_affine);
                    tf::poseMsgToEigen(push_direction, direction_affine);
                    tf::poseMsgToEigen(pose, pose_affine);

                    //compute start and goal pose from affine transforms
                    tf::poseEigenToMsg(pose_affine * direction_affine * start_pose_affine, start_pose);
                    tf::poseEigenToMsg(pose_affine * direction_affine * goal_pose_affine, goal_pose);
                    tf::poseEigenToMsg(pose_affine * direction_affine * retreat_pose_affine, retreat_pose);

                    //fill waypoints
                    std::vector<geometry_msgs::Pose> waypoints;
                    waypoints.push_back(start_pose);
                    visualizePushApproach(push.approach.frame_id, start_pose, 0.0, 1);
                    waypoints.push_back(goal_pose);
                    waypoints.push_back(retreat_pose);
                    visualizePushApproach(push.approach.frame_id, goal_pose, 0.0, 2);
                    //TODO: add retreat waypoint

                    group_.setPoseReferenceFrame(push.approach.frame_id);

                    ROS_INFO_STREAM("Planning push trajectory with start_pose:\n" << start_pose << "\ngoal_pose:\n" <<goal_pose);

                    //Eigen::Affine3d object_frame = scene_->getFrameTransform(push.approach.frame_id);
                    //state.setToIKSolverFrame(start_pose_affine, push.approach.frame_id);
                    geometry_msgs::PoseStamped ps;
                    ps.pose = start_pose;
                    ps.header.frame_id = push.approach.frame_id;

                    tf_listener_.transformPose(group_.getPlanningFrame(), ps, ps);

                    // create start state from IK
                    if (group_.setJointValueTarget(ps)) {
                        state = group_.getJointValueTarget();
                        group_.setStartState(state);
                        //compute cartesian path
                        float success_fraction = group_.computeCartesianPath(waypoints, 0.3, 3, trajectory);
                        group_.setStartStateToCurrentState();
                        group_.clearPoseTargets();
                        if(success_fraction == 1.0) {
                            return true;
                        }
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

            float sampleRandomPushAngle(int degrees=30) {
                float range = M_PI * degrees / 180.0;
                std::normal_distribution<> d{0,0.5};
                return std::max(-range, std::min(range, (float)d(gen)));
            }

            void onDetectObjects(visualization_msgs::Marker marker) {
                if(&marker != marker_ && createCollisionObject(marker)) {
                    marker_ = &marker;
                }
            }

            bool createCollisionObject(visualization_msgs::Marker& marker) {
                if(marker.type == visualization_msgs::Marker::CUBE) {
                    obj_.id = marker.header.frame_id + "_collision";
                    obj_.header.frame_id = marker.header.frame_id;
                    obj_.primitive_poses.resize(1);
                    obj_.primitive_poses[0].orientation.w = 1;
                    //obj_.primitive_poses[0].position.z = 0.0;
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
    moveit::planning_interface::MoveGroupInterface arm("arm");

    tams_ur5_push_execution::Pusher pusher(arm);
    while(ros::ok()) {
        pusher.performRandomPush();
        ros::Duration(3).sleep();
    }
    return 0;
}


