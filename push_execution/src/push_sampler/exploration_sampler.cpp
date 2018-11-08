#include <push_sampler/exploration_sampler.h>

namespace push_sampler
{
  ExplorationSampler::ExplorationSampler() : PushSampler()
  {
    ros::NodeHandle pnh("~");
    pnh.param("min_table_distance", min_table_distance_, MIN_TABLE_DISTANCE);
    pnh.param("safety_range", safety_range_, SAFETY_RANGE);
    pnh.param("emergency_range", emergency_range_, EMERGENCY_RANGE);
    pnh.param("tip_length", tip_length_, TIP_LENGTH);
  }

  bool ExplorationSampler::sampleRandomPush(tams_ur5_push_execution::Push& push)
  {
    if(!sampleSafePushApproach(push.approach, attempts_))
      return false;

    push.distance = sampleRandomPushDistance();
    adjustContactHeight(push);
    return true;
  }

  bool ExplorationSampler::sampleSafePushApproach(tams_ur5_push_execution::PushApproach& approach, int attempts) {
    if(!object_ready_)
      return false;

    geometry_msgs::PoseStamped object_pose;
    object_pose.pose = object_pose_;
    object_pose.header.frame_id = object_frame_;

    tf_listener_.transformPose(reference_frame_, object_pose, object_pose);

    // get distance from object to table_top
    tf::Vector3 obj_vec;
    object_pose.pose.position.z = 0.0;
    tf::pointMsgToTF(object_pose.pose.position, obj_vec);
    float distance = obj_vec.length();

    if(distance < safety_range_) {
      sampleRandomPushApproach(approach);
      return true;
    }
    if(distance < emergency_range_) {
      ROS_WARN("Object outside of SAFETY RANGE. Sampling for pushes towards table center!");
      obj_vec = -obj_vec;

      tf::Quaternion obj_orientation;
      tf::quaternionMsgToTF(object_pose.pose.orientation, obj_orientation);

      tf::Quaternion push_normal;
      tf::Quaternion push_direction;

      double angle_limit = 20.0 / 180.0 * M_PI;
      for(int i = 0; i < attempts; i++) {
        sampleRandomPushApproach(approach);

        // set push approach surface normal
        tf::quaternionMsgToTF(approach.normal, push_normal);

        // set sampled push direction
        push_direction.setRPY(0.0, 0.0, approach.angle);

        tf::Vector3 push_vec(1,0,0);
        push_vec = tf::quatRotate(obj_orientation * push_normal * push_direction, push_vec);
        double angle_towards_table = obj_vec.angle(push_vec);
        if(angle_towards_table < angle_limit || angle_towards_table > (2*M_PI - angle_limit)) {
          return true;
        }
      }
      ROS_ERROR_STREAM("Push experiment aborted! Could not sample valid push pose in " << attempts << " attempts!");
    }
    else
    {
      ROS_ERROR_STREAM("Push experiment aborted! Object is outside of safety range.");
    }
    return false;
  }

  void ExplorationSampler::adjustContactHeight(tams_ur5_push_execution::Push& push)
  {
    double dim_z = shape_.dimensions[2];
    // Pose height is related to box height and tip length
    // By default the tip aligns with the frame of the box.
    // The tip must be lifted in two mutually exclusive cases:
    // 1. The box is too small and therefore the table distance too short.
    push.approach.point.z = std::max(min_table_distance_ - 0.5 * dim_z, 0.0);
    // 2. The box is too high for the tip and might touch the gripper
    push.approach.point.z = std::max(0.5 * dim_z - tip_length_, push.approach.point.z);
  }
}
