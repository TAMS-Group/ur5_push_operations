/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Rice University
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
 *   * Neither the name of the Rice University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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

/* Author: Henning Kayser */

#pragma once

#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/StateValidityChecker.h>

#include <tf/tf.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>

#include <push_planning/conversions.h>

#pragma once

namespace ob = ompl::base;
namespace oc = ompl::control;

moveit_msgs::AttachedCollisionObject getAttachedCollisionObject(const double& x, const double& y, const double& z){
  moveit_msgs::AttachedCollisionObject obj;
  obj.link_name = "s_model_tool0";
  obj.object.header.frame_id = "/table_top";
  obj.object.id = "pushable_object";
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.push_back(x);
  primitive.dimensions.push_back(y);
  primitive.dimensions.push_back(z);
  obj.object.primitives.push_back(primitive);
  return obj;
}

class PushStateValidityChecker : public ob::StateValidityChecker
{
  private:
    const oc::SpaceInformationPtr si_;
    const planning_scene::PlanningScenePtr scene_;
    const double dimX = 0.162;
    const double dimY = 0.23;
    const double dimZ = 0.112;

  public:
    PushStateValidityChecker(const oc::SpaceInformationPtr &si, const planning_scene::PlanningScenePtr scene) : ob::StateValidityChecker(si), si_(si), scene_(scene)
  {

  }

    bool isValid(const ob::State *state) const override
    {
      //    ob::ScopedState<ob::SE2StateSpace>
      // cast the abstract state type to the type we expect
      //const auto *se2state = state->as<ob::SE2StateSpace::StateType>();

      //// extract the first component of the state and cast it to what we expect
      //const auto *pos = se2state->as<ob::RealVectorStateSpace::StateType>(0);

      //// extract the second component of the state and cast it to what we expect
      //const auto *rot = se2state->as<ob::SO2StateSpace::StateType>(1);

      //// check validity of state defined by pos & rot


      //// return a value that is always true but uses the two variables we define, so we avoid compiler warnings
      return si_->satisfiesBounds(state) && !isStateColliding(state);
    }

    bool isStateColliding(const ob::State *state) const
    {
      moveit_msgs::AttachedCollisionObject obj = getAttachedCollisionObject(dimX, dimY, dimZ);
      obj.object.operation = moveit_msgs::CollisionObject::ADD;

      // create object pose
      //const auto *se2state = state->as<ob::SE2StateSpace::StateType>();

      geometry_msgs::Pose pose;
      convertStateToPose(state, pose);
      //pose.position.x = se2state->getX();
      //pose.position.y = se2state->getY();
      pose.position.z = 0.5 * dimZ + 0.001;
      //tf::quaternionTFToMsg(tf::createQuaternionFromYaw(se2state->getYaw()), pose.orientation);

      obj.object.primitive_poses.push_back(pose);
      scene_->processAttachedCollisionObjectMsg(obj);
      return scene_->isStateColliding();
    }
};

