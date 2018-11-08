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

#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_listener.h>
#include <push_sampler/push_sampler.h>

#pragma once

namespace push_sampler
{
  const float MIN_TABLE_DISTANCE = 0.02;
  const float TIP_LENGTH = 0.08;

  // Range to restrict the object on the table
  const float SAFETY_RANGE = 0.03; // Outside of this range the object is pushed towards the center
  const float EMERGENCY_RANGE = 0.3; // Outside of this range the experiment is aborted

  class ExplorationSampler : public PushSampler
  {
    public:
      ExplorationSampler();

      void setReferenceFrame(const std::string& reference_frame) { reference_frame_ = reference_frame; };
      void setObjectFrame(const std::string& object_frame) { object_frame_ = object_frame; };
      void setObjectPose(const geometry_msgs::Pose pose) { object_pose_ = pose; };

      bool sampleRandomPush(tams_ur5_push_msgs::Push& push);
      void adjustContactHeight(tams_ur5_push_msgs::Push& push);

      void setAttemptCount(int attempts) { attempts_ = attempts; };

    private:
      float safety_range_;
      float emergency_range_;

      float min_table_distance_;
      float tip_length_;
      float tip_radius_;

      std::string reference_frame_;
      std::string object_frame_;
      geometry_msgs::Pose object_pose_;

      tf::TransformListener tf_listener_;

      int attempts_ = 100;

      bool sampleSafePushApproach(tams_ur5_push_msgs::PushApproach& approach, int attempts);
  };
}
