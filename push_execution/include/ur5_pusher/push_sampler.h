#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>

#include <tams_ur5_push_execution/Push.h>
#include <tams_ur5_push_execution/PushApproach.h>

#pragma once



namespace ur5_pusher
{

  class PushSampler
  {
    public:
      PushSampler() {};

      void setMarker(const visualization_msgs::Marker& marker);
      void setReferenceFrame(const std::string& reference_frame);

      geometry_msgs::Pose getPoseFromBoxBorder(double p, double dim_x, double dim_y, double dim_z);
      geometry_msgs::Pose sampleConstrainedPoseFromBox(double p, const double &dim_x, const double &dim_y, const double &dim_z);

      virtual bool sampleRandomPush(tams_ur5_push_execution::Push& push);
      void sampleRandomPushApproach(tams_ur5_push_execution::PushApproach& approach);

    protected:
      visualization_msgs::Marker marker_;

      std::random_device rd;
      std::mt19937 gen{rd()};
      std::uniform_real_distribution<double> unif_dist_{0.0,1.0};

      double sampleRandomPushAngle(double range=0.5);
      double sampleRandomPushDistance(double min=0.005, double max=0.03);
      geometry_msgs::Pose sampleRandomPoseFromBox(double dim_x, double dim_y, double dim_z);
  };
}
