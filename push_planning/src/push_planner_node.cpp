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
#include <tams_ur5_push_execution/Push.h>
#include <tams_ur5_push_execution/PushTrajectory.h>
#include <push_planning/PushPlanAction.h>

#include <push_planning/chained_control_sampler.h>
#include <push_planning/push_state_propagator.h>
#include <push_planning/push_state_validity_checker.h>
#include <push_planning/conversions.h>


namespace ob = ompl::base;
namespace oc = ompl::control;

ros::Publisher traj_pub_, graph_pub_;


void publishPushTrajectory(const ompl::control::PathControl& solution)
{
  tams_ur5_push_execution::PushTrajectory traj_msg;
  controlPathToPushTrajectoryMsg(solution, traj_msg);
  traj_pub_.publish(traj_msg);
}

void publishPlannerData(const ompl::base::PlannerData& data) {
  graph_msgs::GeometryGraph graph_msg;
  plannerDataToGraphMsg(data, graph_msg);
  graph_pub_.publish(graph_msg);
}


void spawnCollisionObject() {
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

  class PushPlannerActionServer
  {
    private:
      ros::NodeHandle nh_;

      actionlib::SimpleActionServer<PushPlanAction> as_;

    public:
      PushPlannerActionServer(ros::NodeHandle& nh, const std::string& action_name) :
        nh_(nh),
        as_(nh_, action_name, boost::bind(&PushPlannerActionServer::executeCB, this, _1), false)
    {
      as_.start();
    }

      void executeCB(const PushPlanGoalConstPtr &goal)
      {
        bool success = true;

        PushPlanResult result;
        if (plan(goal, result)) {
          as_.setSucceeded(result);
        } else
          as_.setAborted(result);
      }

      bool plan(const PushPlanGoalConstPtr& goal, PushPlanResult& result)
      {
        // extract goal request
        const std::string& object_id = goal->object_id;
        const geometry_msgs::Pose& start_pose = goal->start_pose;
        const geometry_msgs::Pose& goal_pose = goal->goal_pose;

        // construct a SE2 state space 
        // and set the bounds for the R^2 part of SE(2) state space
        auto space(std::make_shared<ob::SE2StateSpace>());
        ob::RealVectorBounds bounds(2);
        bounds.setLow(-0.3);
        bounds.setHigh(0.3);
        space->setBounds(bounds);

        // create a push control vector space (approach, direction, distance)
        // control vectors are normalized to (0.0,1.0)
        auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 3));
        ob::RealVectorBounds cbounds(3);
        cbounds.setLow(0.0);
        cbounds.setHigh(1.0);
        cspace->setBounds(cbounds);

        // Declare planner setup
        oc::SimpleSetup* setup;
        oc::SpaceInformationPtr si;

        if(true) {

          // (experimental) set directedcontrolsampler with higher sampling count
          si = std::make_shared<oc::SpaceInformation>(cspace->getStateSpace(), cspace);
          //si->setDirectedControlSamplerAllocator(
          //  [](const oc::SpaceInformation* si){ return std::make_shared<oc::SimpleDirectedControlSampler>(si, 50); });
          si->setDirectedControlSamplerAllocator(
              [](const oc::SpaceInformation* si){ return std::make_shared<ChainedControlSampler>(si, 10); });
          setup = (new oc::SimpleSetup(si));
        } else {
          setup = (new oc::SimpleSetup(cspace));
          si = setup->getSpaceInformation();
        }

        bool canSteer = false;
        // set state propagator
        oc::StatePropagatorPtr push_propagator(std::make_shared<PushStatePropagator>(si, canSteer));
        setup->setStatePropagator(push_propagator);

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

        // initialize StateValidityChecker with updated planning scene
        ob::StateValidityCheckerPtr checker(std::make_shared<PushStateValidityChecker>(si, scene));
        setup->setStateValidityChecker(checker);

        // create a start state
        ob::ScopedState<ob::SE2StateSpace> start_state(space);
        convertPoseToState(start_pose, start_state);

        // create goal state
        ob::ScopedState<ob::SE2StateSpace> goal_state(space);
        convertPoseToState(goal_pose, goal_state);

        // set the start and goal states
        setup->setStartAndGoalStates(start_state, goal_state, 0.05);
        setup->setPlanner(std::make_shared<oc::RRT>(si));
        setup->getPlanner()->as<oc::RRT>()->setGoalBias(0.5);
        setup->getPlanner()->as<oc::RRT>()->setIntermediateStates(true);
        si->setMinMaxControlDuration(1.0, 1.0);
        si->setPropagationStepSize(1.0);

        // attempt to solve the planning problem
        if (setup->solve(300.0))
        {
          //setup->getSolutionPath().printAsMatrix(std::cout);
          controlPathToPushTrajectoryMsg(setup->getSolutionPath(), result.trajectory);

          ob::PlannerData data(si);
          setup->getPlannerData(data);
          plannerDataToGraphMsg(data, result.planner_data);
          return true;
        }
        else {
          result.error_message = "No solution found";
          return false;
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

  push_planning::PushPlannerActionServer planner(nh, "/push_plan_action");
  ros::spin();
  return 0;
}
