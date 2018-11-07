#include <random>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>


#include <tams_ur5_push_execution/Push.h>
#include <tams_ur5_push_execution/PushApproach.h>

#pragma once




namespace ur5_pusher
{

  class PushSampler
  {
    public:
      PushSampler() {};

      bool setObject(const visualization_msgs::Marker& marker);
      bool setObject(const moveit_msgs::CollisionObject& object);
      bool setObject(const shape_msgs::SolidPrimitive& shape);


      virtual bool sampleRandomPush(tams_ur5_push_execution::Push& push) const;
      bool sampleRandomPushApproach(tams_ur5_push_execution::PushApproach& approach) const;


      // constrained sampling
      static geometry_msgs::Pose getPoseFromBoxBorder(double p, double dim_x, double dim_y, double dim_z);
      static geometry_msgs::Pose sampleConstrainedPoseFromBox(double p, const double &dim_x, const double &dim_y, const double &dim_z);


    protected:

      shape_msgs::SolidPrimitive shape_;
      bool object_ready_ = false;

      // basic shape sampling
      static double sampleRandomPushAngle(double range=0.5);
      static double sampleRandomPushDistance(double min=0.005, double max=0.03);
      static geometry_msgs::Pose sampleRandomPoseFromBox(double dim_x, double dim_y, double dim_z);
  };
}
