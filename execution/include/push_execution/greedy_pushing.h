#pragma once
#include <push_execution/push_execution.h>
#include <push_prediction/push_predictor.h>
#include <push_sampler/push_sampler.h>
#include <tams_ur5_push_msgs/Push.h>
#include <push_util/util.h>

#include <tf/transform_datatypes.h>

namespace push_msgs = tams_ur5_push_msgs;

namespace push_execution
{

  class GreedyPushing
  {

    PushExecution* execution_;
    push_sampler::PushSampler sampler_;
    push_prediction::PushPredictor predictor_;

    bool sampleTo(const geometry_msgs::Pose& start, const geometry_msgs::Pose& goal, push_msgs::Push& result, bool minimize=true)
    {
      tf::Transform start_tf, pose_tf;
      tf::poseMsgToTF(start, start_tf);
      push_msgs::Push push;
      geometry_msgs::Pose pose;
      double best_distance = se2distance(start, goal);
      for (int i=0;i<100;i++) {
        sampler_.sampleRandomPush(push);
        predictor_.predict(push, pose);
        tf::poseMsgToTF(pose, pose_tf);
        tf::poseTFToMsg(start_tf * pose_tf, pose);
        double distance = se2distance(pose, goal);
        if( minimize ^ distance > best_distance ) {
          best_distance = distance;
          result = push;
        }
      }
    }

    public:

    GreedyPushing(PushExecution* execution)
      : execution_(execution)
    {
      sampler_.setObject(execution->getObjectMarker());
    };

    void pushObjectToGoal(geometry_msgs::Pose start, const geometry_msgs::Pose& goal, double goal_threshold=0.05)
    {
      while (se2distance(start, goal) > goal_threshold)
        pushTo(start, goal, start);
    }

    void pushTo(const geometry_msgs::Pose& start, const geometry_msgs::Pose& goal, geometry_msgs::Pose result, bool minimize=true, double distance=0.0) {
      push_msgs::Push push;
      sampleTo(start, goal, push, minimize);
      if(distance > 0.0)
        push.distance = distance;
      execution_->performPush(push);
      result = execution_->getObjectPose(push.approach.frame_id);
    }
  };
}
