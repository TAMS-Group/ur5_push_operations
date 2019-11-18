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
#include <actionlib/server/simple_action_server.h>

#include <push_execution/pusher.h>
#include <push_execution/push_execution.h>
#include <push_execution/greedy_pushing.h>

#include <tams_ur5_push_msgs/Push.h>
#include <tams_ur5_push_msgs/PusherMovement.h>
#include <tams_ur5_push_msgs/SamplePredictivePush.h>

#include <tams_ur5_push_msgs/ExecutePush.h>


namespace push_execution {

class PushExecutionServer {
  private:

    PushExecution* push_execution_;
    GreedyPushing* greedy_pushing_;

    actionlib::SimpleActionServer<tams_ur5_push_msgs::ExplorePushesAction> explore_pushes_server_;
    actionlib::SimpleActionServer<tams_ur5_push_msgs::MoveObjectAction> move_object_server_;

    ros::ServiceServer point_service_;
    ros::ServiceServer push_execution_service_;

    ros::ServiceClient push_sampler_;



    bool service_busy_ = false;
    bool execute_= false;
    bool take_snapshots_ = false;
    bool record_ft_data_ = false;

    int id_count_ = 0;

    bool isPusherAvailable()
    {
      return push_execution_->isPusherAvailable();
    }

    bool pointAtBox(tams_ur5_push_msgs::PusherMovement::Request& req, tams_ur5_push_msgs::PusherMovement::Response& res)
    {
      res.success = isPusherAvailable() && push_execution_->pointAtBox();
      return true;
    }

    bool executePush(tams_ur5_push_msgs::ExecutePush::Request& req, tams_ur5_push_msgs::ExecutePush::Response& res)
    {
      ROS_ERROR_STREAM("Received push request" << req);
      if(service_busy_) {
        ROS_ERROR_STREAM("ExecutePush request denied, service is busy!");
        return false;
      }

      if(!isPusherAvailable()) {
        ROS_ERROR_STREAM("ExecutePush request denied, pusher is not available!");
        return false;
      }

      service_busy_ = true;
      id_count_++;
      res.result = push_execution_->performPush(req.push, id_count_, execute_);

      if(res.result) {
        ros::Duration(0.5).sleep();

        while (push_execution_->isObjectColliding(0.03)) {
          greedy_pushing_ = new GreedyPushing(push_execution_);
          greedy_pushing_->setDistanceMode(GreedyPushing::DistanceMode::LINEAR);
          geometry_msgs::Pose pose = push_execution_->getObjectPose();
          geometry_msgs::Pose collision_pose = push_execution_->getCollisionObjectPose("collision_object");
          ROS_ERROR_STREAM("Pushing away from collision object!");
          greedy_pushing_->pushTo(pose, collision_pose, pose, false, 0.05);
          res.result = false;
          ros::Duration(0.5).sleep();
        }
      }
      service_busy_ = false;
      return res.result;
    }

    void acceptExplorePushesGoal()
    {
      tams_ur5_push_msgs::ExplorePushesGoal goal = (*explore_pushes_server_.acceptNewGoal());
      tams_ur5_push_msgs::ExplorePushesFeedback feedback;
      tams_ur5_push_msgs::ExplorePushesResult result;
      result.attempts = 0;
      ros::Time start_time = ros::Time::now();
      int success = true;
      int preempted = false;
      if(!service_busy_ && isPusherAvailable()) {
        push_execution_->reset();
        service_busy_ = true;
        int success_count = 0;
        int failed_in_a_row = 0;
        while (goal.samples==0 || success_count < goal.samples) {
          feedback.id = id_count_;

          // preempt goal if canceled
          if(explore_pushes_server_.isPreemptRequested()) {
            ROS_WARN_STREAM("Preempt requested - canceling goal!");
            preempted = true;
            break;
          }

          // perform new attempt and publish feedback
          result.attempts++;
          id_count_++;
          if(push_execution_->performRandomPush(feedback, execute_)) {
            explore_pushes_server_.publishFeedback(feedback);
            success_count++;
            failed_in_a_row = 0;
          //} else if(failed_in_a_row++ == 10) {
          //  ROS_ERROR("Pusher goal action aborted after 10 failed attempts in a row!");
          //  success = false;
          //  break;
          }
        }
      } else {
        // abort since service is not available
        success = false;
      }


      // stop elapsed time
      result.elapsed_time = ros::Time::now() - start_time;

      // send result
      if(success)
        explore_pushes_server_.setSucceeded(result);
      else if(preempted)
        explore_pushes_server_.setPreempted(result);
      else
        explore_pushes_server_.setAborted(result);

      // free service
      service_busy_ = false;
    }

    void acceptMoveObjectGoalGreedy()
    {
      tams_ur5_push_msgs::MoveObjectResult result;
      if(service_busy_) {
        ROS_ERROR("Push Execution cannot move object to goal - service busy!");
        move_object_server_.setAborted(result);
        return;
      }

      if(!isPusherAvailable()) {
        ROS_ERROR("Push Execution cannot move object to goal - Pusher is not available!");
        move_object_server_.setAborted(result);
        return;
      }

      service_busy_ = true;

      tams_ur5_push_msgs::MoveObjectGoal goal = (*move_object_server_.acceptNewGoal());
      greedy_pushing_ = new GreedyPushing(push_execution_);
      greedy_pushing_->setDistanceMode(GreedyPushing::DistanceMode::SE2);
      greedy_pushing_->setPreventCollision(true);
      greedy_pushing_->pushObjectToGoal(goal.target);
      move_object_server_.setSucceeded();
      service_busy_ = false;
    }

    void acceptMoveObjectGoal()
    {
      tams_ur5_push_msgs::MoveObjectGoal goal = (*move_object_server_.acceptNewGoal());
      tams_ur5_push_msgs::MoveObjectFeedback feedback;
      tams_ur5_push_msgs::MoveObjectResult result;
      result.attempts = 0;
      ros::Time start_time = ros::Time::now();
      int success = true;
      int preempted = false;
      if(!service_busy_ && isPusherAvailable()) {
        push_execution_->reset();
        service_busy_ = true;
        int success_count = 0;
        int failed_in_a_row = 0;
        if(goal.object_id.empty()) {
          goal.object_id = push_execution_->getCurrentObjectID();
        }

        // check if goal is reached!
        while (true) {

          // preempt goal if canceled
          if(move_object_server_.isPreemptRequested()) {
            ROS_WARN_STREAM("Preempt requested - canceling goal!");
            preempted = true;
            break;
          }
          // did we fail too much?
          if(failed_in_a_row == 10) {
            ROS_ERROR("Pusher goal action aborted after 10 failed attempts in a row!");
            success = false;
            break;
          }

          // have we reached our goal?
          //double error = getGoalTargetError(goal);
          //ROS_ERROR_STREAM("Error: " << error);
          //if(error < 0.018) {
          //    ROS_ERROR_STREAM("Goal position reached successfully!");
          //    break;
          //}

          // have we reached our goal?
          if(goalReached(goal)) {
            ROS_ERROR_STREAM("Goal position reached successfully!");
            break;
          }

          // perform new attempt and publish feedback
          feedback.id = id_count_;
          result.attempts++;
          id_count_++;

          tams_ur5_push_msgs::Push push;
          if(!getNextPush(goal, push)) {
            ROS_ERROR("Could not sample push for some reason!");
            failed_in_a_row++;
            continue;
          }
          if(push_execution_->performPush(push, feedback, execute_)) {
            move_object_server_.publishFeedback(feedback);
            success_count++;
            failed_in_a_row = 0;
          } else {
            failed_in_a_row++;
          }
        }
        ROS_INFO_STREAM("Moved object to target successfully!");
      } else {
        // abort since service is not available
        success = false;
      }


      // stop elapsed time
      result.elapsed_time = ros::Time::now() - start_time;

      // send result
      if(success)
        move_object_server_.setSucceeded(result);
      else if(preempted)
        move_object_server_.setPreempted(result);
      else
        move_object_server_.setAborted(result);

      // free service
      service_busy_ = false;
    }

    bool goalReached(tams_ur5_push_msgs::MoveObjectGoal& goal) {
      geometry_msgs::Pose obj_pose = push_execution_->getObjectPose(goal.object_id);
      double x_diff = obj_pose.position.x - goal.target.position.x;
      double y_diff = obj_pose.position.y - goal.target.position.y;

      double distance = sqrt(pow(x_diff,2) + pow(y_diff,2));

      tf::Quaternion q_obj;
      tf::Quaternion q_tar;
      tf::quaternionMsgToTF(obj_pose.orientation, q_obj);
      tf::quaternionMsgToTF(goal.target.orientation, q_tar);
      double yaw = std::fmod(tf::getYaw(q_obj.inverse() * q_tar), 2*M_PI);
      double yaw_diff = std::min(yaw, 2*M_PI - yaw);
      return (distance < 0.025 && (std::abs(yaw_diff) < 0.08));
    }


    double getGoalTargetError(tams_ur5_push_msgs::MoveObjectGoal& goal) {
      return getObjectTargetError(push_execution_->getObjectPose(goal.object_id), goal.target);
    }

    double getObjectTargetError(const geometry_msgs::Pose& object_pose, const geometry_msgs::Pose& target) {
      double x_diff = object_pose.position.x - target.position.x;
      double y_diff = object_pose.position.y - target.position.y;
      double distance = sqrt(pow(x_diff,2) + pow(y_diff,2));

      tf::Quaternion q_obj;
      tf::Quaternion q_tar;
      tf::quaternionMsgToTF(object_pose.orientation, q_obj);
      tf::quaternionMsgToTF(target.orientation, q_tar);
      double yaw = std::fmod(tf::getYaw(q_obj.inverse() * q_tar), 2*M_PI);
      double yaw_diff = 0.2 * std::min(yaw, 2*M_PI - yaw);
      // this is not an accurate error estimate since distance and yaw angle are scaled differently
      // However it works for finding a good solution.
      // Example of sufficient error cost: distance<=0.01, yaw<=0.05 => error<=0.1
      // for about equal weighting we calculate 0.1*yaw
      return std::sqrt((pow(distance,2) + pow(yaw_diff, 2)) );
    }


    /*
     * Calls the push sampler service to query a new push
     */
    bool getNextPush(tams_ur5_push_msgs::MoveObjectGoal& goal, tams_ur5_push_msgs::Push& push) {
      tams_ur5_push_msgs::SamplePredictivePush srv;
      srv.request.object_id = goal.object_id;
      srv.request.object_pose = push_execution_->getObjectPose(goal.object_id);
      srv.request.target = goal.target;
      bool success = push_sampler_.call(srv) && srv.response.success;
      push = srv.response.push;
      return success;
    }

  public:

    PushExecutionServer(ros::NodeHandle& nh) : explore_pushes_server_(nh, "explore_pushes_action", true), move_object_server_(nh, "move_object_action", true)
  {
    ros::NodeHandle pnh("~");
    pnh.param("take_snapshots", take_snapshots_, false);
    pnh.param("record_ft_data", record_ft_data_, false);
    pnh.param("execute", execute_, false);
    point_service_ = nh.advertiseService("point_at_box", &PushExecutionServer::pointAtBox, this);
    push_execution_service_ = nh.advertiseService("push_execution", &PushExecutionServer::executePush, this);
    push_sampler_= nh.serviceClient<tams_ur5_push_msgs::SamplePredictivePush>("predictive_push_sampler");

    push_execution_ = new PushExecution();
    isPusherAvailable();
    if(execute_)
    {
      if(take_snapshots_)
        push_execution_->enableSnapshots();
      if(record_ft_data_)
        push_execution_->enableFTData();
    }


    ros::Rate rate(2);
    while(ros::ok()){
      if(!service_busy_) {
        if(explore_pushes_server_.isNewGoalAvailable()) {
          acceptExplorePushesGoal();
        }
        if(move_object_server_.isNewGoalAvailable()) {
          acceptMoveObjectGoalGreedy();
        }
      }
      rate.sleep();
    }
  }
};
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "push_execution_node");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::NodeHandle nh;

  push_execution::PushExecutionServer pes(nh);

  ros::waitForShutdown();

  return 0;
}
