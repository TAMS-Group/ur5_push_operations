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
#include <tams_ur5_push_execution/Push.h>
#include <ur5_pusher/push_approach_sampler.h>

#pragma once

const double dimX = 0.162;
const double dimY = 0.23;
const double dimZ = 0.112;

ur5_pusher::PushApproachSampler* push_sampler_;

void convertControlToPush(const oc::Control *control, tams_ur5_push_execution::Push& push) {
  const double* ctrl = control->as<oc::RealVectorControlSpace::ControlType>()->values;
  geometry_msgs::Pose pose = push_sampler_->getPoseFromBoxBorder(ctrl[0], dimX, dimY, dimZ);
  push.approach.point = pose.position;
  push.approach.normal = pose.orientation;
  // restrict angle to +- 45°
  push.approach.angle = (ctrl[1] - 0.5) * 0.5 * M_PI;
  //push.approach.angle = ctrl[1] - 0.5;
  push.distance = ctrl[2] * 0.05;
}

void convertPoseToState(const geometry_msgs::Pose& pose, ob::ScopedState<ob::SE2StateSpace>& state){
  state->setX(pose.position.x);
  state->setY(pose.position.y);
  state->setYaw(tf::getYaw(pose.orientation));
}

void convertStateToPose(const ob::State *state, geometry_msgs::Pose& pose) {
  const auto *se2state = state->as<ob::SE2StateSpace::StateType>();
  pose.position.x = se2state->getX();
  pose.position.y = se2state->getY();
  pose.position.z = 0.0;
  tf::quaternionTFToMsg(tf::createQuaternionFromYaw(se2state->getYaw()), pose.orientation);
}

void controlPathToPushTrajectoryMsg(const ompl::control::PathControl& solution, tams_ur5_push_execution::PushTrajectory& traj_msg) {
  traj_msg.steps = solution.getStateCount();

  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;

  for(int i = 0; i < traj_msg.steps; i++) {
    pose.position.x = solution.getState(i)->as<ob::SE2StateSpace::StateType>()->getX();
    pose.position.y = solution.getState(i)->as<ob::SE2StateSpace::StateType>()->getY();
    double yaw = solution.getState(i)->as<ob::SE2StateSpace::StateType>()->getYaw();
    Eigen::Quaterniond q(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    tf::quaternionEigenToMsg(q, pose.orientation);
    traj_msg.poses.push_back(pose);

    if (i < solution.getControlCount()) {
      tams_ur5_push_execution::Push push;
      convertControlToPush(solution.getControl(i), push);
      traj_msg.pushes.push_back(push);
    }
  }
}

void plannerDataToGraphMsg(const ompl::base::PlannerData& data, graph_msgs::GeometryGraph& graph_msg) {
  graph_msg.header.frame_id = "table_top";
  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;

  for(unsigned int i = 0; i < data.numVertices(); i++) {

    // fill node positions
    pose.position.x = data.getVertex(i).getState()->as<ob::SE2StateSpace::StateType>()->getX();
    pose.position.y = data.getVertex(i).getState()->as<ob::SE2StateSpace::StateType>()->getY();
    graph_msg.nodes.push_back(pose.position);

    // copy adjacent edges
    graph_msgs::Edges edges;
    data.getEdges(i, edges.node_ids);

    // copy edge weights
    ompl::base::Cost cost;
    for(unsigned int n : edges.node_ids) {
      if(data.getEdgeWeight(i,n,&cost)) {
        edges.weights.push_back(cost.value());
      }
    }

    // fill edges
    graph_msg.edges.push_back(edges);
  }
}
