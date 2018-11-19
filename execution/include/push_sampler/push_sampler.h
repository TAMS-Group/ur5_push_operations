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

#include <random>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>


#include <tams_ur5_push_msgs/Push.h>
#include <tams_ur5_push_msgs/PushApproach.h>

#pragma once




namespace push_sampler
{

  class PushSampler
  {
    public:
      PushSampler() {};

      bool setObject(const visualization_msgs::Marker& marker);
      bool setObject(const moveit_msgs::CollisionObject& object);
      bool setObject(const shape_msgs::SolidPrimitive& shape, const std::string& object_frame="");
      void setObjectFrame(const std::string& object_frame) { object_frame_ = object_frame; };

      virtual bool sampleRandomPush(tams_ur5_push_msgs::Push& push) const;
      bool sampleRandomPushApproach(tams_ur5_push_msgs::PushApproach& approach) const;


      // constrained sampling
      static geometry_msgs::Pose getPoseFromBoxBorder(double p, double dim_x, double dim_y, double dim_z);
      static geometry_msgs::Pose sampleConstrainedPoseFromBox(double p, const double &dim_x, const double &dim_y, const double &dim_z);


    protected:

      shape_msgs::SolidPrimitive shape_;
      std::string object_frame_;
      bool object_ready_ = false;

      // basic shape sampling
      static double sampleRandomPushAngle(double range=0.5);
      static double sampleRandomPushDistance(double min=0.005, double max=0.03);
      static geometry_msgs::Pose sampleRandomPoseFromBox(double dim_x, double dim_y, double dim_z);
  };
}
