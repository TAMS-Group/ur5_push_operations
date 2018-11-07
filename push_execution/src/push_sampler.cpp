#include <ur5_pusher/push_sampler.h>

namespace ur5_pusher
{

  bool PushSampler::setObject(const visualization_msgs::Marker& marker)
  {
    if(marker.type != visualization_msgs::Marker::CUBE) {
      ROS_ERROR_STREAM("Invalid object passed to push sampler - Only BOX/CUBE types are supported!");
      return false;
    }

    shape_.type = shape_msgs::SolidPrimitive::BOX;
    shape_.dimensions.resize(3);
    shape_.dimensions.push_back(marker.scale.x);
    shape_.dimensions.push_back(marker.scale.y);
    shape_.dimensions.push_back(marker.scale.z);
    object_ready_ = true;
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

    return setObject(object.primitives[0]);
  }

  bool PushSampler::setObject(const shape_msgs::SolidPrimitive& shape)
  {
    if(shape.type != shape_msgs::SolidPrimitive::BOX) {
      ROS_ERROR("Invalid object passed to push sampler - Only BOX/CUBE types are supported!");
      return false;
    }
    shape_ = shape;
    object_ready_ = true;
    return true;
  }

  bool PushSampler::sampleRandomPush(tams_ur5_push_execution::Push& push)
  {
    push.distance = sampleRandomPushDistance();
    return sampleRandomPushApproach(push.approach);
  }

  bool PushSampler::sampleRandomPushApproach(tams_ur5_push_execution::PushApproach& approach)
  {
    if(!object_ready_) return false;

    geometry_msgs::Pose approach_pose = sampleRandomPoseFromBox(shape_.dimensions[0],
          shape_.dimensions[1], shape_.dimensions[2]);
    approach.point = approach_pose.position;
    approach.normal = approach_pose.orientation;
    approach.angle = sampleRandomPushAngle();
    return true;
  }

  /**
   * Sample random contact point from box dimensions
   */
  geometry_msgs::Pose PushSampler::sampleRandomPoseFromBox(double dim_x, double dim_y, double dim_z) {
    std::uniform_real_distribution<> dis(0.0, 1.0);
    return getPoseFromBoxBorder(dis(gen), dim_x, dim_y, dim_z);
  }

  /**
   * Sample contact point from box on the same side as push pivot p
   */
  geometry_msgs::Pose PushSampler::sampleConstrainedPoseFromBox(double p, const double &dim_x, const double &dim_y, const double &dim_z) {
    std::uniform_real_distribution<> dis(0.0, 1.0);
    double r = dis(gen);
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
    pose.position.z = 0.5*dim_z;
    return pose;
  }

  double PushSampler::sampleRandomPushAngle(double range) {
    return 2 * range * unif_dist_(gen) - range;
  }

  double PushSampler::sampleRandomPushDistance(double min, double max) {
    return min + unif_dist_(gen) * (max - min);
  }
}
