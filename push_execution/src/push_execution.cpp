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
#include <tams_ur5_push_execution/PerformRandomPush.h>

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

            ur5_pusher::PushApproachSampler push_sampler_;

            tf::TransformListener tf_listener_;

            moveit_msgs::CollisionObject obj_;

            visualization_msgs::Marker marker_;
            ros::Time marker_stamp_;

            bool first_attempt_ = true;

        public:
            PushExecution(bool execute_plan=false) : push_sampler_(SAFETY_RANGE, EMERGENCY_RANGE, MIN_TABLE_DISTANCE, WORKABLE_TIP_LENGTH){

                nh_ = (*new ros::NodeHandle());
                pnh_ = (*new ros::NodeHandle("~"));
                psi_ = (*new moveit::planning_interface::PlanningSceneInterface());

                push_sampler_.setReferenceFrame("/table_top");

                marker_sub_ = nh_.subscribe(MARKER_TOPIC, 1, &PushExecution::onDetectObjects, this);
                contact_point_pub_ = nh_.advertise<visualization_msgs::Marker>("/push_approach", 0);
                marker_.id = -1;
            }

            bool performRandomPush(ur5_pusher::Pusher& pusher, bool execute_plan=false) {
                if(!marker_.header.frame_id.empty()) {
                    if(ros::Time(0) - marker_stamp_ > ros::Duration(0.5)) {
                        ROS_WARN_THROTTLE(10, "Marker not up to date, skipping push");
                        return false;
                    }

                    pusher.setPlanningTime(5.0);
                    pusher.setPlannerId("RRTConnectkConfigDefault");

                    // create push message
                    Push push;
                    createRandomPushMsg(push);

                    //remove collision object in case the last attempt failed
                    std::vector<std::string> object_ids;
                    object_ids.push_back(obj_.id);
                    psi_.removeCollisionObjects(object_ids);

                    // declare plan and start state
                    moveit::planning_interface::MoveGroupInterface::Plan push_plan;
                    robot_state::RobotState start_state(*pusher.getCurrentState());

                    //compute push and retreat trajectory together with start state
                    if(computeCartesianPushTraj(pusher, push, push_plan.trajectory_, start_state)) {

                        // move to pre_push pose on first attempt
                        if(execute_plan) {
                            // apply collision object before moving to pre_push
                            std_msgs::ColorRGBA color;
                            color.r = 0.5;
                            color.a = 0.5;
                            psi_.applyCollisionObject(obj_, color);
                            pusher.setJointValueTarget(start_state);
                            pusher.move();
                            ros::Duration(1.0).sleep();
                        }

                        if(execute_plan)
                            pusher.execute(push_plan);

                        // add collision object after run
                        return true;

                    } else {
                        ROS_INFO_STREAM("Failed to plan and execute push trajectory!");
                    }
                }
                return false;
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

            bool computeCartesianPushTraj(ur5_pusher::Pusher& pusher, tams_ur5_push_execution::Push& push, moveit_msgs::RobotTrajectory& trajectory, robot_state::RobotState& state) {
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
                    pusher.setPoseReferenceFrame("table_top");
                    pusher.setStartStateToCurrentState();
                    double success_fraction = pusher.computeCartesianPushPath(waypoints, 0.05, 3, trajectory);
                    if(success_fraction == 1.0) {
                        trajectory_processing::IterativeParabolicTimeParameterization iptp;
                        robot_trajectory::RobotTrajectory traj(pusher.getRobotModel(), pusher.getName());
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

            bool createCollisionObject(visualization_msgs::Marker& marker, float padding=0.025) {
                if(marker.type == visualization_msgs::Marker::CUBE) {
                    obj_.id = marker.header.frame_id + "_collision";
                    obj_.header.stamp = ros::Time(0);
                    obj_.header.frame_id = marker.header.frame_id;
                    obj_.primitive_poses.resize(1);
                    obj_.primitive_poses[0].orientation.w = 1;
                    obj_.primitive_poses[0].position.z = 0.5 * padding;
                    obj_.primitives.resize(1);
                    obj_.operation = moveit_msgs::CollisionObject::ADD;
                    obj_.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
                    obj_.primitives[0].dimensions.resize(3);
                    obj_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = marker.scale.x + 2 * padding;
                    obj_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = marker.scale.y + 2 * padding;
                    obj_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = marker.scale.z + padding;
                    return true;
                }
                return false;
            }
    };


    class PushExecutionService {
        private:
            PushExecution* push_execution_;
            ros::ServiceServer service_;

            ur5_pusher::Pusher pusher_;

            bool service_busy_ = false;
            bool run_nonstop_= false;
            bool execute_= false;

            bool isPusherAvailable()
            {
                if(!pusher_.isPusherAttached() && !pusher_.loadFromAttachedObject()) {
                    ROS_WARN_STREAM("PushExecutionService is running but no pusher is attached to group '" << pusher_.getName() << "'.");
                    return false;
                }
                return true;
            }

            bool performRandomPush(bool execute) {
                    ROS_INFO("Performing random push");
                    return push_execution_->performRandomPush(pusher_, execute);

            }

        public:

	    bool onPushRequest(tams_ur5_push_execution::PerformRandomPush::Request& req, 
			    tams_ur5_push_execution::PerformRandomPush::Response& res)
	    {
            // First check if the request is of type OP_NONSTOP_TERMINATE
		    if(req.operation == PerformRandomPush::Request::OP_NONSTOP_TERMINATE) { //NONSTOP_TERMINATE
			    execute_ = req.execute;
                run_nonstop_ = false;
                service_busy_ = false;
                res.result = true;
                return true;
            }


            if(!service_busy_) {

                // If service and pusher are available, the request can be handled
                if (isPusherAvailable()) {
                    service_busy_ = true;

                    if(req.operation == PerformRandomPush::Request::OP_DEFAULT) { //DEFAULT
                        res.result = performRandomPush(req.execute);
                        service_busy_ = false;
                    }
                    if(req.operation == PerformRandomPush::Request::OP_NONSTOP) { //Start NONSTOP
                        execute_ = req.execute;
                        run_nonstop_ = true;
                        res.result = true;
                    }
                    return true;

                } else { // Pusher is not available!
                    ROS_ERROR("Unable to perform push operations since no pusher is attached!");
                    res.result = false;
                } 
            } else { // Service is still busy!
                ROS_ERROR_STREAM("PushExecutionService is busy and unable to handle request!");
                res.result = false;
            }

            return false;
        }

        PushExecutionService(ros::NodeHandle& nh, std::string group_name) : pusher_(group_name)
        {
            isPusherAvailable();
            push_execution_ = new PushExecution();
            service_ = nh.advertiseService("/push_execution", &PushExecutionService::onPushRequest, this);

            ROS_INFO("Service advertised!");

            ros::Rate rate(2);
            while(ros::ok()){
                if(run_nonstop_) {
                    if(isPusherAvailable()) {
                        performRandomPush(execute_);
                    } else {
                        ROS_ERROR("Nonstop Push Execution terminated. Pusher is not available anymore!");
                        run_nonstop_ = false;
                        service_busy_ = false;
                    }
                } else {
                    rate.sleep();
                }
            }
        }
    };
}

    int main(int argc, char** argv) {
        ros::init(argc, argv, "push_execution_node");
    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::NodeHandle nh;

    tams_ur5_push_execution::PushExecutionService pes(nh, "arm");

    ros::waitForShutdown();

    return 0;
}


