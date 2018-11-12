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

//ROS
#include <ros/ros.h>
#include <tf/tf.h>
#include <actionlib/server/simple_action_server.h>

// MoveIt
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

// messages
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <graph_msgs/GeometryGraph.h>
#include <shape_msgs/SolidPrimitive.h>
#include <moveit_msgs/CollisionObject.h>

// OMPL
#include <ompl/config.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleDirectedControlSampler.h>

#include <ompl/control/planners/rrt/RRT.h>
// #include <ompl/control/planners/kpiece/KPIECE1.h>
// #include <ompl/control/planners/est/EST.h>
// #include <ompl/control/planners/syclop/SyclopRRT.h>
// #include <ompl/control/planners/syclop/SyclopEST.h>
// #include <ompl/control/planners/pdst/PDST.h>
// #include <ompl/control/planners/syclop/GridDecomposition.h>


// pushing
#include <tams_ur5_push_msgs/PushTrajectory.h>
#include <tams_ur5_push_msgs/PlanPushAction.h>

#include <push_planning/chained_control_sampler.h>
#include <push_planning/push_state_propagator.h>
#include <push_planning/push_state_validity_checker.h>
#include <push_planning/conversions.h>


namespace ob = ompl::base;
namespace oc = ompl::control;
namespace push_msgs = tams_ur5_push_msgs;

ros::Publisher traj_pub_, graph_pub_;


void publishPushTrajectory(const ompl::control::PathControl& solution) 
{
  push_msgs::PushTrajectory traj_msg;
  controlPathToPushTrajectoryMsg(solution, traj_msg);
  traj_pub_.publish(traj_msg);
}

void publishPlannerData(const ompl::base::PlannerData& data)
{
  graph_msgs::GeometryGraph graph_msg;
  plannerDataToGraphMsg(data, graph_msg);
  graph_pub_.publish(graph_msg);
}

void spawnCollisionObject()
{
  moveit::planning_interface::PlanningSceneInterface psi;
  moveit_msgs::CollisionObject cobj;
  cobj.id = "collision_object";
  cobj.header.frame_id = "/table_top";
  shape_msgs::SolidPrimitive primitive;
  cobj.operation = cobj.ADD;
  primitive.type = primitive.BOX;
  primitive.dimensions.push_back(0.03);
  primitive.dimensions.push_back(0.4);
  primitive.dimensions.push_back(0.2);
  cobj.primitives.push_back(primitive);
  geometry_msgs::Pose pose;
  pose.position.z = 0.101;
  pose.position.y = -0.2;
  pose.orientation.w = 1.0;
  cobj.primitive_poses.push_back(pose);
  psi.applyCollisionObject(cobj);
}


namespace push_planning {

  enum ExplorationStrategy { RANDOM, DIRECTED, STEERED, CHAINED };

  class PushPlannerActionServer
  {
    private:
      ros::NodeHandle nh_;
      ros::NodeHandle pnh_;


      actionlib::SimpleActionServer<push_msgs::PlanPushAction> as_;


      ExplorationStrategy strategy_ = CHAINED;

      // planner setup
      double planning_time_ = 300.0;
      double goal_accuracy_ = 0.05;
      double goal_bias_ = 0.5;
      double min_control_duration_ = 1.0;
      double max_control_duration_ = 1.0;
      double propagation_step_size_ = 1.0;
      bool set_intermediate_states_ = true;

      // state space
      double state_space_real_min_ = -0.3;
      double state_space_real_max_ = 0.3;

      // control sampler
      int control_sampler_iterations_ = 10;

      bool can_steer_ = false;


    public:
      PushPlannerActionServer(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& action) :
        nh_(nh),
        pnh_(pnh),
        as_(nh_, action, boost::bind(&PushPlannerActionServer::planCB, this, _1), false)
    {
      loadParams();
      as_.start();
    }

      void loadParams() {
        std::string strategy;
        pnh_.param<std::string>("planning_strategy", strategy, "");
        strategy_ = RANDOM;
        if(strategy == "DIRECTED") strategy_ = DIRECTED;
        else if(strategy == "STEERED") strategy_ = STEERED;
        else if(strategy == "CHAINED") strategy_ = CHAINED;
        else if(strategy != "RANDOM") ROS_WARN("Unknown planning strategy: '%s'", strategy.c_str());

        // planner setup
        pnh_.param("planning_time", planning_time_, 300.0);
        pnh_.param("goal_accuracy", goal_accuracy_, 0.05);
        pnh_.param("goal_bias", goal_bias_, 0.5);
        pnh_.param("min_control_duration", min_control_duration_, 1.0);
        pnh_.param("max_control_duration", max_control_duration_, 1.0);
        pnh_.param("propagation_step_size", propagation_step_size_, 1.0);
        pnh_.param("set_intermediate_states", set_intermediate_states_, true);

        // state space
        pnh_.param("state_space_real_min", state_space_real_min_, -0.3);
        pnh_.param("state_space_real_max", state_space_real_max_, 0.3);

        // control sampler
        pnh_.param("control_sampler_iterations", control_sampler_iterations_, 10);
        can_steer_ = strategy_ == STEERED;
      }


      template <class T> oc::DirectedControlSamplerAllocator getControlSamplerAllocator() {
        return [&](const oc::SpaceInformation* si){ return std::make_shared<T>(si, control_sampler_iterations_); };
      }

      planning_scene::PlanningScenePtr getPlanningScene(){
        // load current planning scene and look for collision objects
        moveit::planning_interface::PlanningSceneInterface psi;
        std::map<std::string, moveit_msgs::CollisionObject> cobjs = psi.getObjects();

        // apply collision objects to planning scene of StateValidityChecker
        planning_scene_monitor::PlanningSceneMonitor psm("robot_description");
        psm.startStateMonitor();
        psm.waitForCurrentRobotState(ros::Time::now());
        planning_scene::PlanningScenePtr scene(psm.getPlanningScene());
        for (auto& cobj : cobjs)
          scene->processCollisionObjectMsg(cobj.second);
        return scene;
      }

      void planCB(const push_msgs::PlanPushGoalConstPtr& goal)
      {

        // extract goal request (not used atm)
        //const std::string& object_id = goal->object_id;

        // construct a SE2 state space 
        // and set the bounds for the R^2 part of SE(2) state space
        auto space(std::make_shared<ob::SE2StateSpace>());
        ob::RealVectorBounds bounds(2);
        bounds.setLow(state_space_real_min_);
        bounds.setHigh(state_space_real_max_);
        space->setBounds(bounds);

        // create a push control vector space (approach, direction, distance)
        // control vectors are normalized to (0.0,1.0)
        auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 3));
        ob::RealVectorBounds cbounds(3);
        cbounds.setLow(0.0);
        cbounds.setHigh(1.0);
        cspace->setBounds(cbounds);

        // create start and goal states
        ob::ScopedState<ob::SE2StateSpace> start_state(space);
        convertPoseToState(goal->start_pose, start_state);
        ob::ScopedState<ob::SE2StateSpace> goal_state(space);
        convertPoseToState(goal->goal_pose, goal_state);

        // Declare planner setup and space information
        oc::SimpleSetup* setup;
        oc::SpaceInformationPtr si;

        // initialize setup and space information
        if(strategy_ == DIRECTED || strategy_ == CHAINED) {
          // custom control samplers need to be allocated within the space information
          // the setup is then initialized with the modified space information

          oc::DirectedControlSamplerAllocator sampler;
          if(strategy_ == DIRECTED)
            sampler = getControlSamplerAllocator<oc::SimpleDirectedControlSampler>();
          if(strategy_ == CHAINED)
            sampler = getControlSamplerAllocator<ChainedControlSampler>();

          si = std::make_shared<oc::SpaceInformation>(cspace->getStateSpace(), cspace);
          si->setDirectedControlSamplerAllocator(sampler);
          setup = (new oc::SimpleSetup(si));

        } else {

          // by default the setup is initialized with the control space
          setup = (new oc::SimpleSetup(cspace));
          si = setup->getSpaceInformation();
        }

        // set state propagator
        oc::StatePropagatorPtr propagator(std::make_shared<PushStatePropagator>(si, can_steer_));
        setup->setStatePropagator(propagator);

        // initialize StateValidityChecker with updated planning scene
        ob::StateValidityCheckerPtr checker(
            std::make_shared<PushStateValidityChecker>(si, getPlanningScene()));
        setup->setStateValidityChecker(checker);

        // configure planner setup
        setup->setStartAndGoalStates(start_state, goal_state, goal_accuracy_);
        setup->setPlanner(std::make_shared<oc::RRT>(si));
        setup->getPlanner()->as<oc::RRT>()->setGoalBias(goal_bias_);
        setup->getPlanner()->as<oc::RRT>()->setIntermediateStates(set_intermediate_states_);
        si->setMinMaxControlDuration(min_control_duration_, max_control_duration_);
        si->setPropagationStepSize(propagation_step_size_);

        // attempt to solve the planning problem
	push_msgs::PlanPushResult result;
        if (setup->solve(planning_time_)) {

          // return solution
          ob::PlannerData data(si);
          setup->getPlannerData(data);
          plannerDataToGraphMsg(data, result.planner_data);
          controlPathToPushTrajectoryMsg(setup->getSolutionPath(), result.trajectory);
          as_.setSucceeded(result);

        } else {

          // return error message
          result.error_message = "No solution found";
          as_.setAborted(result);
        }
      }
  };
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "push_planner_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  if(pnh.param<bool>("spawn_collision_object_test", false))
    spawnCollisionObject();

  push_planning::PushPlannerActionServer planner(nh, pnh, "/push_plan_action");
  ros::spin();
  return 0;
}
