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

#pragma once

#include <math.h>
#include <stdio.h>
#include <boost/variant.hpp>
#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>
#include <chrono>
#include <ctime> 

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <moveit/robot_state/conversions.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/Mesh.h>
#include <Eigen/Geometry>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <push_execution/pusher.h>
#include <tams_ur5_push_msgs/PerformRandomPush.h>
#include <tams_ur5_push_msgs/PusherMovement.h>
#include <tams_ur5_push_msgs/ExplorePushesAction.h>
#include <tams_ur5_push_msgs/MoveObjectAction.h>


std::string COMMAND_MAINTENANCE = "maintenance";
std::string COMMAND_GRIPPER_OPEN = "open";
std::string COMMAND_GRIPPER_CLOSE = "close";
std::string COMMAND_PUSHER_ATTACH = "attach";
std::string COMMAND_PUSHER_DETACH = "detach";
std::string COMMAND_DEMO = "demo";
std::string COMMAND_PUSH = "push";
std::string COMMAND_PUSH_NONSTOP = "push nonstop";
std::string COMMAND_PUSH_BACK = "push back";
std::string COMMAND_PUSH_TARGET = "push target";
std::string COMMAND_SAVE_TARGET = "save target";
std::string COMMAND_POINT = "point";
std::string COMMAND_SET = "set";
std::string COMMAND_UNSET = "unset";
std::string COMMAND_HELP = "help";
std::string COMMAND_QUIT = "quit";

std::string FLAG_EXECUTE = "execute";

std::vector<double> MAINTENANCE_POSITIONS;
std::vector<std::string> JOINT_NAMES;

class PushExecutionClient {
  public:
    ros::NodeHandle nh_;

    actionlib::SimpleActionClient<tams_ur5_push_msgs::ExplorePushesAction> explorer_;
    actionlib::SimpleActionClient<tams_ur5_push_msgs::MoveObjectAction> mover_;


    const std::string FN_PUSHES = "pushes";
    const std::string FN_PRE_POSES = "pre_poses";
    const std::string FN_POST_POSES = "post_poses";

    bool dump_feedback_;
    std::string dump_dir_;

    PushExecutionClient(const std::string& dump_dir) : explorer_("explore_pushes_action"), mover_("move_object_action"), dump_dir_(dump_dir)
  {
    dump_feedback_ = true;
    if(dump_feedback_) {
      //TODO: check if file exists
      write_csv_header_pose(FN_PRE_POSES);
      write_csv_header_push(FN_PUSHES);
      write_csv_header_pose(FN_POST_POSES);
    }
  }

    bool performRandomPushAction(int samples=0)
    {
      explorer_.waitForServer();
      tams_ur5_push_msgs::ExplorePushesGoal goal;
      goal.samples = samples;
      explorer_.sendGoal(goal, &exploreDoneCb, &activeCb, boost::bind(&PushExecutionClient::exploreFeedbackCb, this, _1));
      return true;
    }

    bool moveObjectToPose(const geometry_msgs::Pose& target)
    {
      mover_.waitForServer();
      tams_ur5_push_msgs::MoveObjectGoal goal;
      goal.target = target;
      mover_.sendGoal(goal, &moveDoneCb, &activeCb, boost::bind(&PushExecutionClient::moveFeedbackCb, this, _1));
      return true;
    }

    void abortActions()
    {
      //explorer_.cancelGoal();
      explorer_.cancelAllGoals();
      mover_.cancelAllGoals();
    }

  private:
    static void exploreDoneCb(const actionlib::SimpleClientGoalState& state,
        const tams_ur5_push_msgs::ExplorePushesResultConstPtr& result)
    {
    }

    static void moveDoneCb(const actionlib::SimpleClientGoalState& state,
        const tams_ur5_push_msgs::MoveObjectResultConstPtr& result)
    {
    }

    static void activeCb()
    {
    }

    void exploreFeedbackCb(const tams_ur5_push_msgs::ExplorePushesFeedbackConstPtr& fb)
    {
      if(dump_feedback_) {
        dumpFeedback(fb->id, fb->pre_push, fb->post_push, fb->push);
      }
    }

    void moveFeedbackCb(const tams_ur5_push_msgs::MoveObjectFeedbackConstPtr& fb)
    {
      if(dump_feedback_) {
        dumpFeedback(fb->id, fb->pre_push, fb->post_push, fb->push);
      }
    }

    void dumpFeedback(int id, const geometry_msgs::Pose& pre_push, const geometry_msgs::Pose& post_push, const tams_ur5_push_msgs::Push& push) {
      write_csv_line(FN_PRE_POSES, id, pre_push);
      write_csv_line(FN_PUSHES, id, push);
      write_csv_line(FN_POST_POSES, id, post_push);
    }

    void write_csv_header_pose(const std::string& file_name)
    {
      std::ofstream file(dump_dir_ + "/" + file_name + ".csv");
      file << "id,";
      file << "position.x,";
      file << "position.y,";
      file << "position.z,";
      file << "orientation.x,";
      file << "orientation.y,";
      file << "orientation.z,";
      file << "orientation.w" << std::endl;
      file.close();
    }

    void write_csv_line(const std::string& file_name, int id, const geometry_msgs::Pose& pose)
    {
      std::ofstream file(dump_dir_ + "/" + file_name + ".csv", std::ofstream::out | std::ofstream::app);
      file << std::to_string(id) << ",";
      file << pose.position.x << ",";
      file << pose.position.y << ",";
      file << pose.position.z << ",";
      file << pose.orientation.x << ",";
      file << pose.orientation.y << ",";
      file << pose.orientation.z << ",";
      file << pose.orientation.w << std::endl;
      file.close();
    }

    void write_csv_header_push(const std::string& file_name)
    {
      std::ofstream file(dump_dir_ + "/" + file_name + ".csv");
      file << "id,";
      file << "mode,";
      file << "approach.frame_id,";
      file << "approach.point.x,";
      file << "approach.point.y,";
      file << "approach.point.z,";
      file << "approach.normal.x,";
      file << "approach.normal.y,";
      file << "approach.normal.z,";
      file << "approach.normal.w,";
      file << "push.x,";
      file << "push.y,";
      file << "push.z,";
      file << "approach.angle,";
      file << "distance" << std::endl;
      file.close();
    }

    void write_csv_line(const std::string& file_name, int id, const tams_ur5_push_msgs::Push& push)
    {
      std::ofstream file(dump_dir_ + "/" + file_name + ".csv", std::ofstream::out | std::ofstream::app);
      file << std::to_string(id) << ",";
      file << std::to_string(push.mode) << ",";
      file << push.approach.frame_id << ",";
      file << push.approach.point.x << ",";
      file << push.approach.point.y << ",";
      file << push.approach.point.z << ",";
      file << push.approach.normal.x << ",";
      file << push.approach.normal.y << ",";
      file << push.approach.normal.z << ",";
      file << push.approach.normal.w << ",";
      // represent push as vector
      double x = std::sin(push.approach.angle + 0.5*M_PI) * push.distance;
      double y = std::cos(push.approach.angle - 0.5*M_PI) * push.distance;
      double z = 0.0;
      file << x << ",";
      file << y << ",";
      file << z << ",";
      file << push.approach.angle << ",";
      file << push.distance << std::endl;
      file.close();
    }
};


class PushOperations
{
  private:
    push_execution::Pusher arm_;
    moveit::planning_interface::MoveGroupInterface gripper_;
    moveit::planning_interface::PlanningSceneInterface psi_;

    ros::NodeHandle nh_;

    bool execute_push_operations_ = false;
    PushExecutionClient pec_;
    ros::ServiceClient pusher_movements_;

    tf::TransformListener tf_listener_;
    geometry_msgs::PoseStamped push_target_;

  public:

    PushOperations(const std::string& push_result_dir) : arm_("arm"), gripper_("gripper"), pec_(push_result_dir){

      ros::NodeHandle nh;
      pusher_movements_ = nh.serviceClient<tams_ur5_push_msgs::PusherMovement>("point_at_box");
      createMaintenanceState();

      push_target_.header.frame_id="table_top";
      push_target_.pose.orientation.w = 1.0;

      geometry_msgs::Pose pose;
      pose.position.x = 0.2 - 0.0305;
      pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, -0.5*M_PI, 0.0);

      Eigen::Affine3d transform;
      tf::poseMsgToEigen(pose, transform);

      arm_.setTouchLinks(gripper_.getLinkNames());

      const std::string resource = "package://tams_ur5_push_bringup/meshes/pusher_2_aligned_x-binary.stl";
      std::string parent_link = "s_model_tool0";
      arm_.loadPusher(resource, transform, parent_link, "pusher0");
    };

    void createMaintenanceState()
    {
      //TODO: extract values to config
      MAINTENANCE_POSITIONS.push_back(-1.5708);
      MAINTENANCE_POSITIONS.push_back(-3.1413);
      MAINTENANCE_POSITIONS.push_back(0.0);
      MAINTENANCE_POSITIONS.push_back(1.5708);
      MAINTENANCE_POSITIONS.push_back(1.5708);
      MAINTENANCE_POSITIONS.push_back(0.0);

      JOINT_NAMES.push_back("ur5_shoulder_pan_joint");
      JOINT_NAMES.push_back("ur5_shoulder_lift_joint");
      JOINT_NAMES.push_back("ur5_elbow_joint");
      JOINT_NAMES.push_back("ur5_wrist_1_joint");
      JOINT_NAMES.push_back("ur5_wrist_2_joint");
      JOINT_NAMES.push_back("ur5_wrist_3_joint");
    }


    bool moveToMaintenancePose() {
      // move to initial pose
      robot_state::RobotState state = (*arm_.getCurrentState());
      state.setVariablePositions(JOINT_NAMES, MAINTENANCE_POSITIONS);
      arm_.setJointValueTarget(state);
      return bool(arm_.move());
    }

    bool moveToUpPose(double z_val) {
      // move gripper up to mount pusher
      geometry_msgs::Pose pose;
      pose.position.z = z_val;
      pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, 0.0, 0.0);
      arm_.setPoseReferenceFrame("table_top");
      arm_.setPusherPoseTarget(pose);
      return bool(arm_.move());
    }

    bool moveToDownPose(double z_val) {
      // move gripper up to mount pusher
      geometry_msgs::Pose pose;
      pose.position.z = z_val;
      pose.orientation.w = 1.0;
      arm_.setPoseReferenceFrame("table_top");
      arm_.setPusherPoseTarget(pose);
      return bool(arm_.move());
    }

    bool pointAtBox() {
      tams_ur5_push_msgs::PusherMovement srv;
      pusher_movements_.call(srv);
    }

    bool openGripper() {
      gripper_.setNamedTarget("open");
      gripper_.move();
    }

    bool closeGripper() {
      gripper_.setNamedTarget("closed");
      gripper_.move();
    }

    void importMeshFromResource(shape_msgs::Mesh& mesh_msg) {
      std::string resource = "file:///informatik2/students/home/1kayser/ros_demo/src/tams_ur5_push_operations/tams_ur5_push_bringup/meshes/pusher_2_aligned-binary.stl";
      Eigen::Vector3d scale(0.001, 0.001, 0.001);
      shapes::Shape* shape = shapes::createMeshFromResource(resource, scale);
      shapes::ShapeMsg shape_msg;
      shapes::constructMsgFromShape(shape, shape_msg);
      mesh_msg = boost::get<shape_msgs::Mesh>(shape_msg);
    }

    moveit_msgs::AttachedCollisionObject getAttachedPusher() {
      // create pusher collision object
      moveit_msgs::CollisionObject pusher;
      pusher.header.frame_id = "s_model_palm";
      pusher.id = "pusher0";
      shape_msgs::Mesh mesh;
      importMeshFromResource(mesh);
      pusher.meshes.push_back(mesh);
      geometry_msgs::Pose pose;
      pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.5*M_PI);
      pose.position.y = 0.2 + 0.05625;
      pusher.mesh_poses.push_back(pose);

      // create AttachedCollisionObject
      moveit_msgs::AttachedCollisionObject attached_pusher;
      attached_pusher.link_name = arm_.getEndEffectorLink();
      attached_pusher.object = pusher;
      attached_pusher.touch_links = gripper_.getLinkNames();
      return attached_pusher;
    }

    void attachPusher() {
      arm_.attachPusher();
    }

    void detachPusher() {
      arm_.detachPusher();
    }

    void setExecute(bool execute) {
      execute_push_operations_ = execute;
    }

    void wait(float seconds) {
      ros::Duration(seconds).sleep();
    }

    bool runDemoMovement(int times=0) {
      // compute and perform 8 movement
      moveToDownPose(0.04);
      arm_.setPoseReferenceFrame("table_top");
      //geometry_msgs::Pose pose;
      geometry_msgs::Pose pose;
      pose.orientation.w = 1.0;
      pose.position.z = 0.04;
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      std::vector<geometry_msgs::Pose> waypoints;
      float radius = 0.05;
      double angle = 0.0;
      for(int i = 0; i < 30; i++) {
        angle = 2 * M_PI * i / 50.0;
        pose.position.x = sin(2 * angle) * radius;
        pose.position.y = -sin(angle) * radius;
        waypoints.push_back(pose);
      }
      float success = arm_.computeCartesianPushPath(waypoints, 0.03, 3, plan.trajectory_);
      if(success==1.0) {
        int count = 0;
        // TODO: add keyboard callback
        while((times==0 || count++ <= times)) {
          if (!arm_.execute(plan)) 
            return false;
        }
        return true;
      } else {
        ROS_INFO_STREAM("\nFailed 8 trajectory with percentage " << success);
        return false;
      }
    }

    bool performRandomPushAction(int samples = 100)
    {
      return pec_.performRandomPushAction(samples);
    }

    bool pushObjectBack()
    {
      geometry_msgs::Pose target;
      // rotated around 180° have something in demo mode
      target.orientation.w = 0.0;
      target.orientation.z = 1.0;
      return pec_.moveObjectToPose(target);
    }

    void abortAction()
    {
      pec_.abortActions();
    }

    bool savePushTarget()
    {
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id="pushable_object_0";
      pose.pose.orientation.w = 1.0;
      try{
        tf_listener_.transformPose("table_top", pose, push_target_);
      }
      catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        return false;
      }
      return true;
    }

    bool pushToTarget() 
    {
      return pec_.moveObjectToPose(push_target_.pose);
    }
};
