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
#include <push_execution/push_execution.h>
#include <push_prediction/push_predictor.h>
#include <push_sampler/push_sampler.h>
#include <tams_ur5_push_msgs/Push.h>
#include <push_util/util.h>

#include <tf/transform_datatypes.h>

namespace push_msgs = tams_ur5_push_msgs;

namespace push_execution
{

  class GreedyPushing
  {

    public:

    typedef enum { LINEAR, YAW, SE2 } DistanceMode;

    GreedyPushing(PushExecution* execution)
      : execution_(execution)
    {
      sampler_.setObject(execution->getObjectMarker());
    }

    void setDistanceMode(const DistanceMode mode)
    {
      distance_mode_ = mode;
    }

    void setPreventCollision(bool prevent_collision)
    {
      prevent_collision_ = prevent_collision;
    }

    void pushObjectToGoal(const geometry_msgs::Pose& goal, double goal_threshold=0.05)
    {
      pushObjectToGoal(execution_->getObjectPose(), goal, goal_threshold);
    }

    void pushObjectToGoal(geometry_msgs::Pose start, const geometry_msgs::Pose& goal, double goal_threshold=0.05)
    {
      while (getDistance(start, goal) > goal_threshold) {
        ROS_ERROR_STREAM("Distance: " << getDistance(start, goal));
        pushTo(start, goal, start);

        // let execution update object poses
        ros::Duration(0.5).sleep();

        // if collision prevention is enabled, push away from object
        while(prevent_collision_ && execution_->isObjectColliding(0.03)) {
          ROS_ERROR_STREAM("Found collision - pushing away from object!");
          DistanceMode last_mode = distance_mode_;
          setDistanceMode(GreedyPushing::DistanceMode::LINEAR);
          geometry_msgs::Pose pose;
          pushTo(execution_->getObjectPose(), execution_->getCollisionObjectPose("collision_object"), pose, false, 0.05);
          distance_mode_ = last_mode;
          ros::Duration(0.5).sleep();
        }

        // save update object pose
        start = execution_->getObjectPose();
      }
      ROS_ERROR_STREAM("Done with distance: " << getDistance(start, goal));
    }

    void pushTo(const geometry_msgs::Pose& start, const geometry_msgs::Pose& goal, geometry_msgs::Pose result, bool minimize=true, double distance=0.0) {
      push_msgs::Push push;
      sampleTo(start, goal, push, minimize);
      if(distance > 0.0)
        push.distance = distance;
      execution_->performPush(push);
      result = execution_->getObjectPose(push.approach.frame_id);
    }


    private:

    PushExecution* execution_;
    push_sampler::PushSampler sampler_;
    push_prediction::PushPredictor predictor_;
    DistanceMode distance_mode_ = SE2;
    bool prevent_collision_ = false;

    bool sampleTo(const geometry_msgs::Pose& start, geometry_msgs::Pose goal, push_msgs::Push& result, bool minimize=true)
    {
      // transform goal to start pose
      tf::Transform start_tf, goal_tf;
      tf::poseMsgToTF(start, start_tf);
      tf::poseMsgToTF(goal, goal_tf);
      tf::poseTFToMsg(start_tf.inverse() * goal_tf, goal);

      push_msgs::Push push;
      geometry_msgs::Pose pose;
      pose.orientation.w = 1.0;
      double best_distance = getDistance(pose, goal);
      for (int i=0;i<100;i++) {
        sampler_.sampleRandomPush(push);
        predictor_.predict(push, pose);
        double distance = getDistance(pose, goal);
        if( minimize ^ distance > best_distance ) {
          best_distance = distance;
          result = push;
        }
      }
    }

    double getDistance(const geometry_msgs::Pose& start, const geometry_msgs::Pose& goal){
	    if(distance_mode_ == LINEAR)
		    return linearDistance(start, goal);
	    if(distance_mode_ == YAW)
		    return yawDistance(start, goal);
	    if(distance_mode_ == SE2)
		    return se2Distance(start, goal);
    }
  };
}
