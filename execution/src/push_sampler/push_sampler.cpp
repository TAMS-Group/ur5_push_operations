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
#include <push_sampler/push_sampler.h>

std::random_device rd;
std::mt19937 gen{rd()};
std::uniform_real_distribution<double> unif_dist_{0.0,1.0};
namespace push_sampler
{
  PushSampler::PushSampler()
  {
    ros::NodeHandle pnh("~");
    pnh.param("min_push_distance", min_push_distance_, 0.005);
    pnh.param("min_push_distance", max_push_distance_, 0.03);
    pnh.param("push_angle_range", push_angle_range_, 0.5);
  }

  bool PushSampler::setObject(const visualization_msgs::Marker& marker)
  {
    if(marker.type != visualization_msgs::Marker::CUBE) {
      ROS_ERROR_STREAM("Invalid object passed to push sampler - Only BOX/CUBE types are supported!");
      return false;
    }

    shape_.type = shape_msgs::SolidPrimitive::BOX;
    shape_.dimensions.resize(3);
    shape_.dimensions[0] = marker.scale.x;
    shape_.dimensions[1] = marker.scale.y;
    shape_.dimensions[2] = marker.scale.z;
    object_ready_ = true;
    object_frame_ = marker.header.frame_id;
    return true;
  }

  bool PushSampler::setObject(const moveit_msgs::CollisionObject& object)
  {
    if(object.primitives.size() != 1) {
      ROS_ERROR("Invalid object passed to push sampler - Collision object has invalid number of primitive shapes, should be 1!");
      return false;
    }
    if(object.primitives[0].type != shape_msgs::SolidPrimitive::BOX) {
      ROS_ERROR("Invalid object passed to push sampler - Only BOX/CUBE types are supported!");
      return false;
    }

    return setObject(object.primitives[0], object.header.frame_id);
  }

  bool PushSampler::setObject(const shape_msgs::SolidPrimitive& shape, const std::string& object_frame)
  {
    if(shape.type != shape_msgs::SolidPrimitive::BOX) {
      ROS_ERROR("Invalid object passed to push sampler - Only BOX/CUBE types are supported!");
      return false;
    }
    shape_ = shape;
    object_frame_ = object_frame;
    object_ready_ = true;
    return true;
  }

  bool PushSampler::sampleRandomPush(tams_ur5_push_msgs::Push& push) const
  {
    push.distance = sampleRandomPushDistance(min_push_distance_, max_push_distance_);
    return sampleRandomPushApproach(push.approach);
  }

  bool PushSampler::sampleRandomPushApproach(tams_ur5_push_msgs::PushApproach& approach) const
  {
    if(!object_ready_) return false;

    geometry_msgs::Pose approach_pose =
      sampleRandomPoseFromBox(shape_.dimensions[0],
                              shape_.dimensions[1],
                              shape_.dimensions[2]);

    approach.frame_id = object_frame_;
    approach.point = approach_pose.position;
    approach.normal = approach_pose.orientation;
    approach.angle = sampleRandomPushAngle(push_angle_range_);
    return true;
  }

  /**
   * Sample random contact point from box dimensions
   */
  geometry_msgs::Pose PushSampler::sampleRandomPoseFromBox(double dim_x, double dim_y, double dim_z) {
    return getPoseFromBoxBorder(unif_dist_(gen), dim_x, dim_y, dim_z);
  }

  /**
   * Sample contact point from box on the same side as push pivot p
   */
  geometry_msgs::Pose PushSampler::sampleConstrainedPoseFromBox(double p, const double &dim_x, const double &dim_y, const double &dim_z) {
    double r = unif_dist_(gen);
    if(p <= dim_x) {
      p = r * dim_x;
    } else if (p <= dim_x + dim_y) {
      p = dim_x + r * dim_y;
    } else if (p <= 2 * dim_x + dim_y) {
      p = dim_x + dim_y + r * dim_x;
    } else {
      p = dim_x + dim_y + dim_x + r * dim_y;
    }
    return getPoseFromBoxBorder(p, dim_x, dim_y, dim_z);
  }

  geometry_msgs::Pose PushSampler::getPoseFromBoxBorder(double p, double dim_x, double dim_y, double dim_z) {
    p = p * 2 * (dim_x + dim_y);
    geometry_msgs::Pose pose;
    // Match value with edge and create corresponding pose
    if(p <= dim_x) {
      pose.position.x = p;
      pose.position.y = 0;
      pose.orientation = tf::createQuaternionMsgFromYaw(0.5*M_PI);
    } else if (p <= dim_x + dim_y) {
      pose.position.x = dim_x;
      pose.position.y = p - dim_x;
      pose.orientation = tf::createQuaternionMsgFromYaw(M_PI);
    } else if (p <= 2 * dim_x + dim_y) {
      pose.position.x = 2 * dim_x + dim_y - p;
      pose.position.y = dim_y;
      pose.orientation = tf::createQuaternionMsgFromYaw(1.5*M_PI);
    } else {
      pose.position.x = 0;
      pose.position.y = 2 * (dim_x + dim_y) - p;
      pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    }

    // move box to center
    pose.position.x -= 0.5*dim_x;
    pose.position.y -= 0.5*dim_y;
    pose.position.z = 0.0;
    return pose;
  }

  double PushSampler::getBoxApproachPivotFromPush(const tams_ur5_push_msgs::Push& push, double dim_x, double dim_y, double dim_z) {
    geometry_msgs::Point point = push.approach.point;
    point.x += 0.5 * dim_x;
    point.y += 0.5 * dim_y;

    // Match value with edge and create corresponding pose
    if(point.y == 0.0)
      return point.x;
    else if (point.x == dim_x)
      return point.y + dim_x;
    else if (point.y == dim_y)
      return 2 * dim_x + dim_y - point.x;
    else
      return 2 * (dim_x + dim_y) - point.y;
  }

  double PushSampler::sampleRandomPushAngle(double range) {
    return 2 * range * unif_dist_(gen) - range;
  }

  double PushSampler::sampleRandomPushDistance(double min, double max) {
    return min + unif_dist_(gen) * (max - min);
  }
}
