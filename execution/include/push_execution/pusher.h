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

#include <algorithm>

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <tf/transform_listener.h>

#include <eigen_conversions/eigen_msg.h>

#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/Mesh.h>


#include <visualization_msgs/Marker.h>

#pragma once

namespace push_execution
{

  class Pusher : public moveit::planning_interface::MoveGroupInterface
  {

    private:
      moveit::planning_interface::PlanningSceneInterface psi_;

      std::string mesh_resource_;
      float mesh_scale_;
      Eigen::Isometry3d tip_transform_;
      moveit_msgs::AttachedCollisionObject pusher_object_;
      std::string parent_link_;
      std::string pusher_id_;
      std::vector<std::string> touch_links_;

      bool pusher_attached_ = false;
      bool knows_pusher_ = false;

      bool loadPusher();
      moveit_msgs::AttachedCollisionObject getPusherObjectMsg(const shape_msgs::Mesh& mesh_msg, const geometry_msgs::Pose& pose_msg) const;
      void importMeshFromResource(const std::string& resource, shape_msgs::Mesh& mesh_msg, float scale) const;

      bool getSingleAttachedObject(moveit_msgs::AttachedCollisionObject& object);

      bool hasSingleAttachedObject();

    public:
      Pusher(const std::string& group_name);

      Pusher(const std::string& group_name, const std::string& resource, const Eigen::Isometry3d& transform, const std::string& parent_link, const std::string& pusher_id);

      void setPusherMeshResource(const std::string& resource, float scale=0.001);

      void setPusherTipTransform(const Eigen::Isometry3d transform);

      void setPusherParentLink(const std::string& parent_link);

      void setPusherId(const std::string& pusher_id);

      void setTouchLinks(const std::vector<std::string>& touch_links);

      bool loadPusher(const std::string& resource, const Eigen::Isometry3d& transform, const std::string& parent_link, const std::string& pusher_id);

      bool loadFromAttachedObject();

      bool isPusherAttached();

      bool knowsPusher() const;

      bool attachPusher();

      bool detachPusher();

      bool setPusherPoseTarget(const geometry_msgs::Pose& pose);

      bool setPusherPoseTarget(const Eigen::Isometry3d& pose);

      bool setPusherJointValueTarget(const geometry_msgs::PoseStamped& pose_stamped);

      bool setPusherJointValueTarget(const geometry_msgs::Pose& pose);

      bool setPusherJointValueTarget(const Eigen::Isometry3d& pose);

      double computeCartesianPushPath(std::vector<geometry_msgs::Pose>& waypoints, double eef_step, double jump_threshold, moveit_msgs::RobotTrajectory& trajectory);
  };
}
