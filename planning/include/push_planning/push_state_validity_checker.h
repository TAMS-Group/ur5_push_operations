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

namespace ob = ompl::base;
namespace oc = ompl::control;

static moveit_msgs::AttachedCollisionObject createObject() {
  moveit_msgs::AttachedCollisionObject obj;
  obj.link_name = "s_model_tool0";
  obj.object.header.frame_id = "/table_top";
  obj.object.id = "pushable_object";
  obj.object.primitive_poses.resize(1);
  obj.object.primitives.resize(1);
  obj.object.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  obj.object.primitives[0].dimensions.push_back(0.162);
  obj.object.primitives[0].dimensions.push_back(0.23);
  obj.object.primitives[0].dimensions.push_back(0.112);
  obj.object.operation = moveit_msgs::CollisionObject::ADD;
  return obj;
}

// this should be initialized from shape
static moveit_msgs::AttachedCollisionObject obj = createObject();


class PushStateValidityChecker : public ob::StateValidityChecker
{
  private:
    const oc::SpaceInformationPtr si_;
    const planning_scene::PlanningScenePtr scene_;

  public:
    PushStateValidityChecker(const oc::SpaceInformationPtr &si, const planning_scene::PlanningScenePtr scene)
      : ob::StateValidityChecker(si), si_(si), scene_(scene)
    {  }

    bool isValid(const ob::State *state) const override
    {
      return si_->satisfiesBounds(state) && !isStateColliding(state);
    }

    bool isStateColliding(const ob::State *state) const
    {
      // move object to state and check for collisions
      convertStateToPose(state, obj.object.primitive_poses[0]);
      obj.object.primitive_poses[0].position.z = 0.5 * obj.object.primitives[0].dimensions[2] + 0.001;
      scene_->processAttachedCollisionObjectMsg(obj);
      return scene_->isStateColliding();
    }
};
