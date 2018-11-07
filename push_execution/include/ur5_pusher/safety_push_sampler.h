#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_listener.h>
#include <ur5_pusher/push_sampler.h>

#pragma once

namespace ur5_pusher
{
  const float MIN_TABLE_DISTANCE = 0.02;
  const float TIP_LENGTH = 0.08;

  // Range to restrict the object on the table
  const float SAFETY_RANGE = 0.03; // Outside of this range the object is pushed towards the center
  const float EMERGENCY_RANGE = 0.3; // Outside of this range the experiment is aborted

  class SafetyPushSampler : public PushSampler
  {
    public:
      SafetyPushSampler();
      void setReferenceFrame(const std::string& reference_frame);
      bool sampleRandomPush(tams_ur5_push_execution::Push& push);
      void adjustContactHeight(tams_ur5_push_execution::Push& push);

      void setAttemptCount(int attempts) { attempts_ = attempts; };

    private:
      float safety_range_;
      float emergency_range_;

      float min_table_distance_;
      float tip_length_;
      float tip_radius_;

      std::string reference_frame_;
      tf::TransformListener tf_listener_;

      int attempts_ = 100;

      bool sampleSafePushApproach(tams_ur5_push_execution::PushApproach& approach, int attempts);
  };
}
