/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Lars Henning Kayser
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Lars Henning Kayser */

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ExecuteTrajectoryActionResult.h>

#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_state/conversions.h>

#include <tf/transform_listener.h>

#include <eigen_conversions/eigen_msg.h>

#include <visualization_msgs/Marker.h>

#include <push_execution/pusher.h>
#include <push_sampler/exploration_sampler.h>

#include <tams_ur5_push_msgs/ExplorePushesAction.h>
#include <tams_ur5_push_msgs/MoveObjectAction.h>
#include <tams_ur5_push_msgs/Push.h>
#include <tams_ur5_push_msgs/PushApproach.h>
#include <tams_ur5_push_msgs/ImageDump.h>
#include <tams_ur5_push_msgs/FTDump.h>

#include <std_msgs/Float64.h>
#include <XmlRpcValue.h>
#include <XmlRpcException.h>

#pragma once



const double TIP_RADIUS = 0.004;
const double RETREAT_HEIGHT = 0.05;
const double APPROACH_DISTANCE = 0.05;

std::string MARKER_TOPIC = "/pushable_objects";

namespace push_msgs = tams_ur5_push_msgs;

namespace push_execution
{
    class PushExecution
    {
        private:

            ros::NodeHandle nh_;
            ros::NodeHandle pnh_;

            ros::Subscriber marker_sub_;
            ros::Publisher contact_point_pub_;

            planning_scene_monitor::PlanningSceneMonitor psm_;
            planning_scene_monitor::CurrentStateMonitorPtr csm_;

            ros::Publisher dist_pub_;
            ros::Publisher dist2_pub_;

            moveit::planning_interface::PlanningSceneInterface psi_;

            push_sampler::ExplorationSampler push_sampler_;
            Pusher pusher_;

            ros::ServiceClient snapshot_client_;
            ros::ServiceClient ft_client_;
            bool take_snapshots_ = false;
            bool record_ft_data_ = false;

            tf::TransformListener tf_listener_;

            moveit_msgs::CollisionObject obj_;

            visualization_msgs::Marker marker_;
            ros::Time marker_stamp_;

            bool first_attempt_ = true;

            double tip_radius_;
            double retreat_height_;
            double approach_distance_;

        public:
            PushExecution(bool execute_plan=false, const std::string& group_name="arm") : psm_("robot_description"), pusher_(group_name){

                nh_ = (*new ros::NodeHandle());
                pnh_ = (*new ros::NodeHandle("~"));
                psi_ = (*new moveit::planning_interface::PlanningSceneInterface());
                psm_.startStateMonitor();
                csm_ = psm_.getStateMonitorNonConst();
                dist_pub_ = nh_.advertise<std_msgs::Float64>("/joint_distances/contact", 100);
                dist2_pub_ = nh_.advertise<std_msgs::Float64>("/joint_distances/post_push", 100);

                pnh_.param("tip_radius", tip_radius_, TIP_RADIUS);
                pnh_.param("retreat_height", retreat_height_, RETREAT_HEIGHT);
                pnh_.param("approach_distance", approach_distance_, APPROACH_DISTANCE);
                int object_id;
                pnh_.param<int>("object_id", object_id, 0);
                ros::NodeHandle nh;
                XmlRpc::XmlRpcValue objects;
                nh_.getParam("objects", objects);
                if (object_id > 0 && objects.size() > object_id)
                {
                  try {
                    if (objects[object_id].begin()->second.hasMember("retreat_height"))
                      retreat_height_ = objects[object_id].begin()->second["retreat_height"];
                    if (objects[object_id].begin()->second.hasMember("approach_distance"))
                      approach_distance_ = objects[object_id].begin()->second["approach_distance"];
                  } catch (XmlRpc::XmlRpcException& e) {
                    ROS_WARN_STREAM("Error extracting values of object " << object_id << " from configuration file!");
                    ROS_WARN("%s", e.getMessage().c_str());
                  }
                }

                push_sampler_.setReferenceFrame("/table_top");

                marker_sub_ = nh_.subscribe(MARKER_TOPIC, 1, &PushExecution::onDetectObjects, this);
                contact_point_pub_ = nh_.advertise<visualization_msgs::Marker>("/push_approach", 0);
                marker_.id = -1;
            }

            bool performRandomPush(bool execute_plan=false) {
                push_msgs::ExplorePushesFeedback fb;
                return performRandomPush(fb, execute_plan);
            }

            void enableSnapshots() {
                snapshot_client_ = nh_.serviceClient<push_msgs::ImageDump>("/image_dump_service");
                take_snapshots_ = true;
            }

            void disableSnapshots() {
                take_snapshots_ = false;
            }

            void enableFTData() {
                ft_client_ = nh_.serviceClient<push_msgs::FTDump>("/ft_dump_service");
                record_ft_data_ = true;
            }

            void disableFTData() {
                record_ft_data_ = false;
            }

            bool performRandomPush(push_msgs::ExplorePushesFeedback& feedback, bool execute_plan=true)
            {
                push_msgs::Push push;
                ros::Duration(0.5).sleep();
                if (isObjectClear() && createRandomPushMsg(push)) {
                    try {
                        feedback.push = push;
                        tf::poseEigenToMsg(getObjectTransform(push.approach.frame_id), feedback.pre_push);
                        bool success = performPush(push, feedback.id, execute_plan);
                        // Observe Relocation
                        tf::poseEigenToMsg(getObjectTransform(push.approach.frame_id), feedback.post_push);
                        return success;
                    }
                    catch (tf::TransformException transform_exception){
                        ROS_ERROR_STREAM("Invalid transform exception in push execution " << transform_exception.what());
                    }
                }
                return false;
            }

            bool performPush(const push_msgs::Push& push, push_msgs::MoveObjectFeedback& feedback, bool execute_plan=true)
            {
                ros::Duration(0.5).sleep();
                if (isObjectClear()) {
                    try {
                        feedback.push = push;
                        tf::poseEigenToMsg(getObjectTransform(push.approach.frame_id), feedback.pre_push);
                        bool success = performPush(push, feedback.id, execute_plan);
                        // Observe Relocation
                        tf::poseEigenToMsg(getObjectTransform(push.approach.frame_id), feedback.post_push);
                        return success;
                    }
                    catch (tf::TransformException transform_exception){
                        ROS_ERROR_STREAM("Invalid transform exception in push execution " << transform_exception.what());
                    }
                }
                return false;
            }

            bool performPush(const push_msgs::Push& push, int attempt_id=0, bool execute_plan=true)
            {
                if(isObjectClear()) {

                    pusher_.setPlannerId("RRTConnectkConfigDefault");
                    pusher_.setPlanningTime(5.0);

                    //remove collision object in case the last attempt failed
                    if (!removeCollisionObject())
                    {
                        ROS_ERROR_STREAM("Unable to remove collision object " << obj_.id);
                        return false;
                    }

                    // declare plan and start state
                    moveit::planning_interface::MoveGroupInterface::Plan push_plan;
                    moveit_msgs::RobotTrajectory approach_traj, push_traj, retreat_traj;

                    robot_state::RobotState rstate(*pusher_.getCurrentState());

                    std::vector<std::vector<geometry_msgs::Pose>> waypoints;
                    std::vector<double> distances;

                    getPushWaypoints(push, waypoints, distances);

                    // start state to be set from IK
                    geometry_msgs::PoseStamped ps;
                    ps.pose = waypoints.front().front();
                    ps.header.frame_id = "table_top";
                    tf_listener_.transformPose(pusher_.getPlanningFrame(), ps, ps);

                    if(pusher_.setPusherJointValueTarget(ps)) {
                        robot_state::RobotState start_state = pusher_.getJointValueTarget();

                        // apply collision object before moving to pre_push
                        if( !applyCollisionObject()) {
                            ROS_ERROR_STREAM("Unable to set object as collision object in planning scene!");
                            return false;
                        }
                        if(!first_attempt_)
                            pusher_.setPathConstraints(get_pusher_down_constraints());

                        // plan move to box
                        moveit::planning_interface::MoveGroupInterface::Plan move_to_box;
                        pusher_.setStartStateToCurrentState();
                        if(!pusher_.plan(move_to_box)) {
                            ROS_ERROR("Failed planning of movement to push object!");
                            return false;
                        }

                        bool is_executing = true;
                        bool can_push = false;
                        int trajectory_execution_result_code = 0;
                        ros::Subscriber sub = nh_.subscribe<moveit_msgs::ExecuteTrajectoryActionResult>("/execute_trajectory/result", 1,
                                [&] (const moveit_msgs::ExecuteTrajectoryActionResultConstPtr& msg)
                                    {
                                        if(is_executing) {
                                            is_executing = false;
                                            trajectory_execution_result_code = msg->result.error_code.val;
                                            if(can_push) { // if already finished planning - have a short break!
                                                ros::Duration(0.5).sleep();
                                            }
                                        }
                                    });

                        if(!pusher_.asyncExecute(move_to_box)) {
                            ROS_ERROR("Failed execution of movement to box!");
                            return false;
                        }

                        // remove all path constraints
                        pusher_.clearPathConstraints();

                        // allow object collision
                        if (!removeCollisionObject())
                        {
                            ROS_ERROR_STREAM("Unable to remove collision object " << obj_.id);
                            return false;
                        }

                        // plan push
                        can_push = computeCartesianPushTraj(waypoints, distances, approach_traj, push_traj, retreat_traj, start_state);

                        // wait for trajectory to finish
                        while(is_executing) {
                            ROS_INFO_THROTTLE(1, "Moving to approach pose.");
                        }
                        pusher_.setStartStateToCurrentState();
                        if(trajectory_execution_result_code != moveit_msgs::MoveItErrorCodes::SUCCESS) {
                            ROS_ERROR("Failed at trajectory execution! Aborting push attempt.");
                            return false;
                        }
                        if(!can_push) {
                            ROS_ERROR("Failed to plan push trajectory - Moving back!");
                            robot_trajectory::RobotTrajectory traj(pusher_.getRobotModel(), pusher_.getName());
                            moveit::planning_interface::MoveGroupInterface::Plan move_back;
                            traj.setRobotTrajectoryMsg((*pusher_.getCurrentState()), move_to_box.trajectory_);
                            traj.reverse();
                            traj.getRobotTrajectoryMsg(move_back.trajectory_);
                            recomputeTimestamps(move_back.trajectory_);
                            applyCollisionObject();
                            pusher_.execute(move_back);
                            return false;
                        }

                        // move to pre_push pose on first attempt
                        if(execute_plan && can_push) {

                            // create intermediate states (contact,  post push)
                            robot_state::RobotState contact_state(rstate);
                            contact_state.setJointGroupPositions(pusher_.getName(), approach_traj.joint_trajectory.points.back().positions);

                            if(take_snapshots_) {

                                bool contact_shot = false;

                                // joint state callback
                                csm_->addUpdateCallback(
                                        [&] (const sensor_msgs::JointStateConstPtr& joint_state) {
                                        if(!contact_shot && joint_state->name.size()>0 && joint_state->name[0] == "ur5_shoulder_pan_joint") {
                                        rstate.setJointGroupPositions(pusher_.getName(), joint_state->position);

                                        if(rstate.distance(contact_state) < 0.03) {
                                        contact_shot = true;
                                        take_snapshot(std::to_string(attempt_id) + "_2_contact");
                                        }
                                        }
                                        });
                            }

                            // concatenate trajectory parts
                            pusher_.getCurrentState()->copyJointGroupPositions("arm", approach_traj.joint_trajectory.points[0].positions);
                            robot_trajectory::RobotTrajectory traj(pusher_.getRobotModel(), pusher_.getName());
                            robot_trajectory::RobotTrajectory append_traj(pusher_.getRobotModel(), pusher_.getName());
                            traj.setRobotTrajectoryMsg((*pusher_.getCurrentState()), approach_traj);
                            append_traj.setRobotTrajectoryMsg(contact_state, push_traj);
                            traj.append(append_traj, 0.0);
                            // perform iterative time parameterization
                            traj.getRobotTrajectoryMsg(push_plan.trajectory_);
                            recomputeTimestamps(push_plan.trajectory_);

                            if(take_snapshots_)
                                take_snapshot(std::to_string(attempt_id) + "_1_before");
                            if(record_ft_data_)
                                record_ft_data("ft_data_" + std::to_string(attempt_id), true);

                            // EXECUTE PUSH
                            pusher_.execute(push_plan);

                            if(take_snapshots_)
                                csm_->clearUpdateCallbacks();

                            if(take_snapshots_)
                                take_snapshot(std::to_string(attempt_id) + "_3_after");

                            if(record_ft_data_)
                                record_ft_data("ft_data_" + std::to_string(attempt_id), false);

                            // retreat trajectory
                            pusher_.getCurrentState()->copyJointGroupPositions("arm", retreat_traj.joint_trajectory.points[0].positions);
                            recomputeTimestamps(retreat_traj);
                            push_plan.trajectory_ = retreat_traj;

                            pusher_.execute(push_plan);

                            first_attempt_ = false;
                            return true;
                        }
                    } else {
                        ROS_INFO_STREAM("Failed to plan and execute push trajectory!");
                    }
                }
                return false;
            }

            bool pointAtBox() {
                if(marker_.header.frame_id.empty())
                    return false;
                geometry_msgs::Pose pose;
                pose.position.z = 0.5 * marker_.scale.z + 0.025;
                pose.orientation.w = 1.0;
                pusher_.setPoseReferenceFrame(marker_.header.frame_id);
                pusher_.setPusherPoseTarget(pose);
                applyCollisionObject();
                bool success = bool(pusher_.move());
                return removeCollisionObject();
            }

            void reset()
            {
                first_attempt_ = true;
            }

            geometry_msgs::Pose getObjectPose()
            {
                return getObjectPose(marker_.header);
            }

            geometry_msgs::Pose getObjectPose(const std::string& frame_id, const std::string& target_frame="table_top")
            {
                std_msgs::Header obj_header;
                obj_header.frame_id = frame_id;
                return getObjectPose(obj_header, target_frame);
            }

            geometry_msgs::Pose getObjectPose(const std_msgs::Header& obj_pose_header, const std::string& target_frame="table_top")
            {
                ros::Time now = ros::Time::now();
                geometry_msgs::PoseStamped obj_pose;
                obj_pose.header = obj_pose_header;
                obj_pose.header.stamp = now;
                obj_pose.pose.orientation.w = 1.0;
                tf_listener_.waitForTransform(target_frame, obj_pose.header.frame_id, now, ros::Duration(1.0));
                tf_listener_.transformPose(target_frame, obj_pose, obj_pose);
                return obj_pose.pose;
            }

            geometry_msgs::Pose getCollisionObjectPose(const std::string& obj_id, const std::string& target_frame="table_top")
            {
                moveit::planning_interface::PlanningSceneInterface psi;
                geometry_msgs::Pose pose = psi.getObjectPoses({ obj_id })[obj_id];
                std::string base_frame = pusher_.getPlanningFrame();
                if(base_frame != target_frame) {
                  tf::Transform target_to_base, base_to_object;
                  tf::poseMsgToTF(getObjectPose(base_frame, target_frame), target_to_base);
                  tf::poseMsgToTF(pose, base_to_object);
                  tf::poseTFToMsg(target_to_base * base_to_object, pose);
                }
                return pose;
            }

            Eigen::Isometry3d getObjectTransform(const std::string& obj_frame, const std::string& target_frame="table_top")
            {
                Eigen::Isometry3d obj_pose_affine;
                std_msgs::Header obj_header;
                obj_header.frame_id = obj_frame;
                obj_header.stamp = ros::Time::now();
                tf::poseMsgToEigen(getObjectPose(obj_header, target_frame), obj_pose_affine);
                return obj_pose_affine;
            }

            std::string getCurrentObjectID() {
                return marker_.header.frame_id;
            }

            visualization_msgs::Marker getObjectMarker() const {
              return marker_;
            }

            bool isObjectColliding(float padding=0.02) {

              psm_.waitForCurrentRobotState(ros::Time::now());
              planning_scene::PlanningScenePtr scene(psm_.getPlanningScene());

              moveit::planning_interface::PlanningSceneInterface psi;
              std::map<std::string, moveit_msgs::CollisionObject> cobjs = psi.getObjects();

              for (auto& cobj : cobjs) {
                if(cobj.first.find(obj_.id) > 1)
                  scene->processCollisionObjectMsg(cobj.second);
              }

              moveit_msgs::AttachedCollisionObject obj;
              createCollisionObject(marker_, obj.object, padding);
              obj.object.id = "padded_collision_test_object";
              obj.link_name = "s_model_tool0";
              obj.object.header.frame_id = "table_top";
              obj.object.primitive_poses[0] = getObjectPose();
              obj.object.primitive_poses[0].position.z += 0.001 + 0.5 * padding;
              scene->processAttachedCollisionObjectMsg(obj);

              bool collision = scene->isStateColliding();
              std::vector<std::string> links;
              scene->getCollidingLinks(links);
              if(links.size() > 0) {
                ROS_ERROR("Colliding links:");
                for (auto link : links)
                  ROS_ERROR_STREAM("" << link);
              }
              obj.object.operation = moveit_msgs::CollisionObject::REMOVE;
              scene->processAttachedCollisionObjectMsg(obj);
              return collision;
            }

            bool isPusherAvailable()
            {
              if(!pusher_.isPusherAttached() && !pusher_.loadFromAttachedObject()) {
                return false;
              }
              return true;
            }

        private:

            bool isObjectClear() {
                // check if we have an object
                if (marker_.header.frame_id.empty()) {
                    ROS_WARN_THROTTLE(10, "Could not find any pushable object! Skipping current push.");
                    return false;
                }
                // check if the objects frame is up to date
                if(ros::Time(0) - marker_stamp_ > ros::Duration(0.5)) {
                    ROS_WARN_THROTTLE(10, "Object frame not up to date! Skipping current push.");
                    return false;
                }
                return true;
            }

            void recomputeTimestamps(moveit_msgs::RobotTrajectory& trajectory_msg, double max_vel=3.14, double max_accel=1.0) {

                robot_trajectory::RobotTrajectory traj(pusher_.getRobotModel(), pusher_.getName());
                traj.setRobotTrajectoryMsg((*pusher_.getCurrentState()), trajectory_msg);
                //trajectory_processing::IterativeSplineParameterization isp;
                trajectory_processing::IterativeParabolicTimeParameterization isp;
                isp.computeTimeStamps(traj, max_vel, max_accel);
                traj.getRobotTrajectoryMsg(trajectory_msg);
            }

            bool applyCollisionObject() {
                std_msgs::ColorRGBA color;
                color.r = 0.5;
                color.a = 0.5;
                return psi_.applyCollisionObject(obj_, color);
            }

            bool removeCollisionObject() {
                if (!psi_.getObjects({obj_.id}).empty())
                {
                    moveit_msgs::CollisionObject obj;
                    obj.id = obj_.id;
                    obj.operation = moveit_msgs::CollisionObject::REMOVE;
                    return psi_.applyCollisionObject(obj);
                }
                ROS_INFO_STREAM("Can't remove object " << obj_.id << ". It's not present in the planning scene");
                return true;
            }

            bool createRandomPushMsg(push_msgs::Push& push) {
                push.mode = push_msgs::Push::LINEAR;
                push_sampler_.setObject(marker_);
                push_sampler_.setObjectPose(marker_.pose);
                push_sampler_.setObjectFrame(marker_.header.frame_id);

                if(push_sampler_.sampleRandomPush(push))
                {
                    visualizePushApproach(push.approach);
                    return true;
                }
                return false;
            }

            void take_snapshot(const std::string& filename)
            {
                push_msgs::ImageDump srv;
                srv.request.filename = filename;
                snapshot_client_.call(srv);
            }

            void record_ft_data(const std::string& filename, bool enabled)
            {
                push_msgs::FTDump srv;
                srv.request.filename = filename;
                srv.request.enabled = enabled;
                ft_client_.call(srv);
            }

            moveit_msgs::Constraints get_pusher_down_constraints() {
                //Create orientation constraint - pusher down
                moveit_msgs::OrientationConstraint ocm;
                ocm.link_name = "s_model_tool0";
                ocm.header.frame_id = "table_top";
                ocm.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0.5*M_PI, 0);
                ocm.absolute_x_axis_tolerance = 0.3;
                ocm.absolute_y_axis_tolerance = 0.3;
                ocm.absolute_z_axis_tolerance = 0.1;
                ocm.weight = 1.0;

                moveit_msgs::Constraints constraints;
                constraints.orientation_constraints.push_back(ocm);
                constraints.name = "pusher:down:0.3:0.1";
                return constraints;
            }

            void visualizePushApproach(const push_msgs::PushApproach& approach) {
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

                Eigen::Isometry3d offset_affine;
                Eigen::Isometry3d direction_affine;
                Eigen::Isometry3d pose_affine;
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



            bool getPushWaypoints(const push_msgs::Push& push, std::vector<std::vector<geometry_msgs::Pose>>& waypoints, std::vector<double>& wp_distances) {

                // object pose
                geometry_msgs::PoseStamped obj_pose;
                obj_pose.header.frame_id = push.approach.frame_id;
                obj_pose.pose.orientation.w = 1.0;
                tf_listener_.transformPose("table_top", obj_pose, obj_pose);
                Eigen::Isometry3d obj_pose_affine;
                tf::poseMsgToEigen(obj_pose.pose, obj_pose_affine);

                // push direction
                Eigen::Isometry3d direction(Eigen::AngleAxis<double>(push.approach.angle, Eigen::Vector3d::UnitZ()));

                // approach point
                Eigen::Isometry3d approach_affine(Eigen::Isometry3d::Identity());
                geometry_msgs::Pose pose;
                pose.position = push.approach.point;

                // move approach pose outwards by tip_radius to disable early contact
                pose.position.x -= tip_radius_;
                pose.orientation = push.approach.normal;
                tf::poseMsgToEigen(pose, approach_affine);
                approach_affine = obj_pose_affine * approach_affine * direction;

                //trajectory distances
                float approach_distance = approach_distance_;
                float retreat_height = marker_.scale.z + retreat_height_;

                // fill waypoints
                geometry_msgs::Pose start_wp, approach_wp, push_wp, retreat_low_wp, retreat_high_wp;
                geometry_msgs::Quaternion orientation;
                orientation.w = 1.0;

                Eigen::Isometry3d wp(Eigen::Isometry3d::Identity()); // instantiate as identity transform!!!
                wp.translate(Eigen::Vector3d(-approach_distance, 0.0, 0.0));
                tf::poseEigenToMsg(approach_affine * wp, start_wp);
                start_wp.orientation = orientation;

                wp.translate(Eigen::Vector3d(approach_distance, 0.0, 0.0));
                tf::poseEigenToMsg(approach_affine * wp, approach_wp);
                approach_wp.orientation = orientation;

                wp.translate(Eigen::Vector3d(push.distance, 0.0, 0.0));
                tf::poseEigenToMsg(approach_affine * wp, push_wp);
                push_wp.orientation = orientation;

                wp.translate(Eigen::Vector3d(-0.015, 0.0, 0.0));
                tf::poseEigenToMsg(approach_affine * wp, retreat_low_wp);
                retreat_low_wp.orientation = orientation;

                wp.translate(Eigen::Vector3d(0.0, 0.0, retreat_height));
                tf::poseEigenToMsg(approach_affine * wp, retreat_high_wp);
                retreat_high_wp.orientation = orientation;

                std::vector<geometry_msgs::Pose> start_wps = {start_wp};
                std::vector<geometry_msgs::Pose> approach_wps = {approach_wp};
                std::vector<geometry_msgs::Pose> push_wps = {push_wp};
                std::vector<geometry_msgs::Pose> retreat_wps = {retreat_low_wp, retreat_high_wp};

                waypoints = {start_wps, approach_wps, push_wps, retreat_wps};

                double retreat_distance = std::sqrt(std::pow(push.distance,2) + std::pow(retreat_height, 2));
                wp_distances = {approach_distance, push.distance, retreat_distance};
            }

            bool computeCartesianPushTraj(std::vector<std::vector<geometry_msgs::Pose>> waypoints, const std::vector<double> distances, moveit_msgs::RobotTrajectory& approach_traj, moveit_msgs::RobotTrajectory& push_traj, moveit_msgs::RobotTrajectory& retreat_traj, const robot_state::RobotState start_state)
            {
                ros::Time start_time = ros::Time::now();
                bool success = false;
                if(waypoints.size() == 4 && distances.size() == 3) {
                    pusher_.setPoseReferenceFrame("table_top");

                    robot_state::RobotStatePtr next_state = std::make_shared<robot_state::RobotState>(start_state);
                    const moveit::core::JointModelGroup* jmg = start_state.getJointModelGroup(pusher_.getName());

                    pusher_.setStartState(start_state);
                    std::vector<geometry_msgs::Pose> wp_target;
                    std::vector<moveit_msgs::RobotTrajectory> trajectories;

                    success = true;
                    int i;
                    double success_fraction;
                    for(i=1; i < waypoints.size(); i++) {
                        moveit_msgs::RobotTrajectory traj;
                        success_fraction = pusher_.computeCartesianPushPath(waypoints[i], 0.1 * distances[i-1], 3, traj);
                        if(success_fraction == 1.0) {
                            trajectories.push_back(traj);
                            next_state->setJointGroupPositions(jmg, traj.joint_trajectory.points.back().positions);
                            pusher_.setStartState((*next_state));
                        } else {
                            ROS_ERROR_STREAM("Failed planning push trajectory step " << i+1 << " with success percentage " << success_fraction * 100 << "%");
                            success = false;
                            break;
                        }
                    }
                    if(success) {
                        ros::Duration planning_time = ros::Time::now() - start_time;
                        ROS_INFO_STREAM("ComputePushTrajectory finished after " << planning_time.toSec() << " seconds.");
                        approach_traj = trajectories[0];
                        push_traj = trajectories[1];
                        retreat_traj = trajectories[2];
                    }
                    /*
                    //compute cartesian path
                    pusher.setStartState(start_state);
                    std::vector<geometry_msgs::Pose> wp_target = {waypoints[1]};
                    double approach_success = pusher.computeCartesianPushPath(wp_target, 0.1 * approach_distance, 3, approach_traj);
                    if(approach_success == 1.0) {
                    robot_state::RobotStatePtr rstate = pusher.getCurrentState();
                    moveit_msgs::RobotState wp_state;
                    const moveit::core::JointModelGroup* jmg = rstate->getJointModelGroup(pusher.getName());
                    rstate->setJointGroupPositions(jmg, approach_traj.joint_trajectory.points.back().positions);
                    moveit::core::robotStateToRobotStateMsg(*rstate, wp_state);
                    pusher.setStartState(wp_state);
                    wp_target = {waypoints[2]};
                    double push_success = pusher.computeCartesianPushPath(wp_target, 0.1 * push.distance, 3, push_traj);
                    if(push_success == 1.0) {
                    rstate->setJointGroupPositions(jmg, push_traj.joint_trajectory.points.back().positions);
                    moveit::core::robotStateToRobotStateMsg(*rstate, wp_state);
                    pusher.setStartState(wp_state);
                    wp_target = {waypoints[3]};
                    double retreat_success = pusher.computeCartesianPushPath(wp_target, 0.1*retreat_distance, 3, retreat_traj);
                    if(retreat_success == 1.0) {
                    success = true;
                    ros::Duration planning_time = ros::Time::now() - start_time;
                    ROS_ERROR_STREAM("ComputePushTrajectory finished after " << planning_time.toSec() << " seconds.");
                    } else {
                    ROS_ERROR_STREAM("Failed planning cartesian retreat path: " << retreat_success * 100 << "%");
                    }
                    } else {
                    ROS_ERROR_STREAM("Failed planning cartesian push path: " << push_success * 100 << "%");
                    }
                    } else {
                    ROS_ERROR_STREAM("Failed planning cartesian approach path: " << approach_success * 100 << "%");
                    }
                     */
                } else {
                    ROS_ERROR_STREAM("Failed to plan push trajectory. Number of waypoints or distances is invalid!");
                }
                return success;
            }

            void onDetectObjects(visualization_msgs::Marker marker) {
                if(marker_.id != marker.id && createCollisionObject(marker, obj_)) {
                    marker_ = marker;
                }
                marker_stamp_ = marker.header.stamp;
            }

            bool createCollisionObject(const visualization_msgs::Marker& marker, moveit_msgs::CollisionObject& obj, float padding=0.015) {
                if(marker.type == visualization_msgs::Marker::CUBE) {
                    obj.id = marker.header.frame_id + "_collision";
                    obj.header.stamp = ros::Time(0);
                    obj.header.frame_id = marker.header.frame_id;
                    obj.primitive_poses.resize(1);
                    obj.primitive_poses[0].orientation.w = 1;
                    obj.primitive_poses[0].position.z = 0.5 * padding;
                    obj.primitives.resize(1);
                    obj.operation = moveit_msgs::CollisionObject::ADD;
                    obj.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
                    obj.primitives[0].dimensions.resize(3);
                    obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = marker.scale.x + 2 * padding;
                    obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = marker.scale.y + 2 * padding;
                    obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = marker.scale.z + padding;
                    return true;
                }
                return false;
            }
    };
}
