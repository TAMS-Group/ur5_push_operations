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

#include <ompl/control/StatePropagator.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

#include <push_prediction/push_predictor.h>
#include <push_planning/conversions.h>



namespace push_planning {

  class PushStatePropagator : public oc::StatePropagator
  {
    private:

      const oc::SpaceInformationPtr si_;
      const oc::ControlSamplerPtr cs_;

      push_prediction::PushPredictor* const predictor_;

      const bool canSteer_;

      bool set_distance_from_duration_ = false;

    public:

      PushStatePropagator(const oc::SpaceInformationPtr &si, push_prediction::PushPredictor& predictor, bool canSteer=false) : oc::StatePropagator(si),
      si_(si),
      cs_(si->allocControlSampler()),
      canSteer_(canSteer),
      predictor_(&predictor)
    {
      predictor_->setReuseSolutions(true);
    }

      /*
         void propagate(const ob::State *start, const oc::Control *control, const double duration, ob::State *result) const override
         {
         bool success = true;

         const double* ctrl = control->as<oc::RealVectorControlSpace::ControlType>()->values;

      // extract start state
      const auto *se2state = start->as<ob::SE2StateSpace::StateType>();
      const double x = se2state->getX();
      const double y = se2state->getY();
      const double yaw = se2state->getYaw();


      tams_ur5_push_msgs::PredictPush msg;
      msg.request.control.push_back(ctrl[0]);
      msg.request.control.push_back(ctrl[1]);
      msg.request.control.push_back(ctrl[2]);

      // predict push control effect
      push_predictor_->call(msg);

      if (msg.response.success) {

      geometry_msgs::Pose np = msg.response.next_pose;
      geometry_msgs::Quaternion q = np.orientation;
      Eigen::Quaterniond next_q(q.w, q.x, q.y, q.z);
      double next_yaw = next_q.toRotationMatrix().eulerAngles(0,1,2)[2];

      // create new state
      Eigen::Affine2d next_pos = Eigen::Translation2d(x, y) 
       * Eigen::Rotation2Dd(yaw) * Eigen::Translation2d(np.position.x, np.position.y);

      // set result state
      result->as<ob::SE2StateSpace::StateType>()->setXY(
      next_pos.translation().x(),
      next_pos.translation().y());
      result->as<ob::SE2StateSpace::StateType>()->setYaw( std::fmod(yaw + next_yaw + M_PI , 2 * M_PI) - M_PI);

      } else {
      ROS_ERROR_STREAM("Predict Push service call failed!");
      result->as<ob::SE2StateSpace::StateType>()->setXY(x,y);
      result->as<ob::SE2StateSpace::StateType>()->setYaw(yaw);
      success = false;
      }
      }
       */

      void propagate(const ob::State *start, const oc::Control *control, const double duration, ob::State *result) const override
      {
        const double* ctrl = control->as<oc::RealVectorControlSpace::ControlType>()->values;

        // extract start state
        const auto *se2state = start->as<ob::SE2StateSpace::StateType>();
        const double x = se2state->getX();
        const double y = se2state->getY();
        const double yaw = se2state->getYaw();

        tams_ur5_push_msgs::Push push;
        convertControlToPush(control, push);
        if(set_distance_from_duration_)
          push.distance = duration;

        // predict push control effect
        geometry_msgs::Pose pose;
        predictor_->predict(push, pose);

        tf::getYaw(pose.orientation);

        geometry_msgs::Quaternion q = pose.orientation;
        Eigen::Quaterniond next_q(q.w, q.x, q.y, q.z);
        double next_yaw = next_q.toRotationMatrix().eulerAngles(0,1,2)[2];

        // create new state
        Eigen::Affine2d next_pos = Eigen::Translation2d(x, y) 
          * Eigen::Rotation2Dd(yaw) * Eigen::Translation2d(pose.position.x, pose.position.y);

        // set result state
        result->as<ob::SE2StateSpace::StateType>()->setXY(
            next_pos.translation().x(),
            next_pos.translation().y());
        result->as<ob::SE2StateSpace::StateType>()->setYaw( std::fmod(yaw + next_yaw + M_PI , 2 * M_PI) - M_PI);
      }

      void se2StateToEigen(const ob::State *state, Eigen::Affine2d& pose) const
      {
        const auto *se2state = state->as<ob::SE2StateSpace::StateType>();
        pose.setIdentity();
        pose.translate(Eigen::Vector2d(se2state->getX(), se2state->getX()));
        pose.rotate(Eigen::Rotation2Dd(se2state->getYaw()));
      }

      double se2Distance(const Eigen::Affine2d& start, const Eigen::Affine2d& goal) const
      {
        Eigen::Affine2d diff = start.inverse() * goal;
        return diff.translation().norm() + 0.5 * Eigen::Rotation2Dd(diff.rotation()).angle();
      }

      bool steer(const ob::State *start, const ob::State *goal, oc::Control *control, double& duration) const override
      {
        return steer2(start, goal, control, duration);
      }

      //bool steerFromSide(const ob::State *start, const ob::State *goal, oc::Control *control, double& duration) const override
      //{
      //    return steer1(start, goal, control, duration);
      //}


      /*
       * Implementation of steer_1:
       * Sample random controls (push steps) and propagate as long as the goal distance decreases.
       * If start + n * push reaches a state that is within goal_threshold, return control and duration as step count.
       */
      bool steer1(const ob::State *start, const ob::State *goal, oc::Control *control, double& duration) const 
      {
        Eigen::Affine2d start_pose;
        se2StateToEigen(start, start_pose);

        Eigen::Affine2d goal_pose;
        se2StateToEigen(goal, goal_pose);

        // temp variables
        tams_ur5_push_msgs::Push push;
        geometry_msgs::Pose pose;

        const double goal_distance = si_->distance(start, goal);
        const double goal_threshold = 0.05;

        Eigen::Affine2d next_pose, step;

        for(int i = 0; i < 100; i++) {

          // sample control
          cs_->sample(control);
          convertControlToPush(control, push);

          // predict sampled push
          predictor_->predict(push, pose);
          step.setIdentity();
          step.translate(Eigen::Vector2d(pose.position.x, pose.position.y));
          step.rotate(Eigen::Rotation2Dd(tf::getYaw(pose.orientation)));

          // compute push step
          next_pose = start_pose * step;
          double next_distance = se2Distance(next_pose, goal_pose);

          duration = 0.0;
          double min_distance = goal_distance;
          while (next_distance < min_distance) {
            duration = duration + 1.0;
            min_distance = next_distance;
            next_pose = next_pose * step;
            next_distance = se2Distance(next_pose, goal_pose);
          }

          if(duration > 0.0 && min_distance < goal_threshold) {
            return true;
          }
        }
        return false;
      }

      /*
       * Implementation of steer_1:
       * Sample random controls (push steps) and propagate as long as the goal distance decreases.
       * If start + n * push reaches a state that is within goal_threshold, return control and duration as step count.
       */
      bool steer2(const ob::State *start, const ob::State *goal, oc::Control *control, double& duration) const 
      {
        Eigen::Affine2d start_pose;
        se2StateToEigen(start, start_pose);

        Eigen::Affine2d goal_pose;
        se2StateToEigen(goal, goal_pose);

        // temp variables
        tams_ur5_push_msgs::Push push;
        geometry_msgs::Pose pose;

        const double goal_threshold = 0.05;

        Eigen::Affine2d step;

        for(int i = 0; i < 100; i++) {

          // sample control
          cs_->sample(control);
          convertControlToPush(control, push);

          // predict sampled push
          predictor_->predict(push, pose);
          step.setIdentity();
          step.translate(Eigen::Vector2d(pose.position.x, pose.position.y));
          step.rotate(Eigen::Rotation2Dd(tf::getYaw(pose.orientation)));


          if(se2Distance(start_pose * step, goal_pose) < goal_threshold) {
            duration = control->as<oc::RealVectorControlSpace::ControlType>()->values[2];
            return true;
          }
        }
        return false;
      }



      bool canPropagateBackward()
      {
        return false;
      }

      bool canSteer() const override
      {
        return canSteer_;
      }

      /*
       * Computes control and duration for a given start/goal state pair.
       * The SteerPush service samples random controls and selects the one with the minimal expected
       * goal distance. The distance is computed as defined by SE2StateSpace, as a weighted sum of arclength (0.5)
       * and cartesian distance (1.0).
       bool steer(const ob::State *start, const ob::State *to, oc::Control *control, double& duration) const override
       {
       tams_ur5_push_msgs::SteerPush msg;
       convertStateToPose(start, msg.request.start);
       convertStateToPose(to, msg.request.goal);
       steer_predictor_->call(msg);
       duration = msg.response.duration;
       auto *rcontrol = control->as<oc::RealVectorControlSpace::ControlType>();
       if (msg.response.control.size() == si_->getControlSpace()->getDimension()) {
       for(int i = 0; i < msg.response.control.size(); i++) {
       rcontrol->values[i] = msg.response.control[i];
       }
       }
       return msg.response.success;
       }
       */
  };
}
