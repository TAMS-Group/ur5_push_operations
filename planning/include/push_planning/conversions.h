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

// Eigen
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

// OMPL
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

// messages
#include <geometry_msgs/Pose.h>

// pushing
#include <tams_ur5_push_msgs/Push.h>
#include <push_sampler/push_sampler.h>

#include <cmath>

#pragma once

const double dimX = 0.162;
const double dimY = 0.23;
const double dimZ = 0.112;
const std::string object_frame = "pushable_object_0";

namespace ob = ompl::base;
namespace oc = ompl::control;

void convertControlToPush(const oc::Control *control, tams_ur5_push_msgs::Push& push) {
  const double* ctrl = control->as<oc::RealVectorControlSpace::ControlType>()->values;
  //retrieve approach point from pivot and box dimensions
  geometry_msgs::Pose pose = push_sampler::PushSampler::getPoseFromBoxBorder(ctrl[0], dimX, dimY, dimZ);
  push.approach.point = pose.position;
  push.approach.normal = pose.orientation;
  push.approach.frame_id = object_frame;
  // normalize angle to +- 45°
  push.approach.angle = (ctrl[1] - 0.5) * 0.5 * M_PI;
  // normalize distance range to 0.0m-0.05m
  push.distance = ctrl[2] * 0.05;
}

void convertPushToControl(const tams_ur5_push_msgs::Push& push, oc::RealVectorControlSpace::ControlType *ctrl) {
  //retrieve approach point from pivot and box dimensions
  ctrl->values[1] = push_sampler::PushSampler::getBoxApproachPivotFromPush(push, dimX, dimY, dimZ);

  // normalize angle to +- 45°
  ctrl->values[1] = std::fmax(0.0, std::fmin(push.approach.angle * 2 / M_PI + 0.5, 1.0));
  // normalize distance range to 0.0m-0.05m
  ctrl->values[2] = std::fmax(0.0, std::fmin(push.distance / 0.05, 1.0));
}

void convertPoseToState(const geometry_msgs::Pose& pose, ob::ScopedState<ob::SE2StateSpace>& state){
  state->setX(pose.position.x);
  state->setY(pose.position.y);
  state->setYaw(tf::getYaw(pose.orientation));
}

void convertStateToPoint(const ob::State *state, geometry_msgs::Point& point) {
  const auto *se2state = state->as<ob::SE2StateSpace::StateType>();
  point.x = se2state->getX();
  point.y = se2state->getY();
  point.z = 0.0;
}

void convertStateToPose(const ob::State *state, geometry_msgs::Pose& pose) {
  const auto *se2state = state->as<ob::SE2StateSpace::StateType>();
  convertStateToPoint(se2state, pose.position);
  tf::quaternionTFToMsg(tf::createQuaternionFromYaw(se2state->getYaw()), pose.orientation);
}


void controlPathToPushTrajectoryMsg(const ompl::control::PathControl& solution, tams_ur5_push_msgs::PushTrajectory& traj_msg) {
  traj_msg.steps = solution.getStateCount();
  traj_msg.poses.resize(traj_msg.steps);
  traj_msg.pushes.resize(solution.getControlCount());

  for(int i = 0; i < traj_msg.steps; i++) {
    convertStateToPose(solution.getState(i), traj_msg.poses[i]);

    if ( i < traj_msg.pushes.size() )
      convertControlToPush(solution.getControl(i), traj_msg.pushes[i]);
  }
}

void plannerDataToGraphMsg(const ompl::base::PlannerData& data, graph_msgs::GeometryGraph& graph_msg) {
  graph_msg.header.frame_id = "table_top";
  graph_msg.nodes.resize(data.numVertices());

  for(size_t i = 0; i < graph_msg.nodes.size(); i++) {

    // fill node positions
    convertStateToPoint(data.getVertex(i).getState(), graph_msg.nodes[i]);

    // copy adjacent edges
    graph_msgs::Edges edges;
    data.getEdges(i, edges.node_ids);

    // copy edge weights
    ompl::base::Cost cost;
    for(unsigned int n : edges.node_ids) {
      if(data.getEdgeWeight(i,n,&cost))
        edges.weights.push_back(cost.value());
    }

    // fill edges
    graph_msg.edges.push_back(edges);
  }
}
