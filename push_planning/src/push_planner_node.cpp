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

/* Author: Ioan Sucan */

#include <ompl/control/SpaceInformation.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/est/EST.h>
#include <ompl/control/planners/syclop/SyclopRRT.h>
#include <ompl/control/planners/syclop/SyclopEST.h>
#include <ompl/control/planners/pdst/PDST.h>
#include <ompl/control/planners/syclop/GridDecomposition.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>
#include <iostream>

#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

// ROS dependencies

#include <ros/ros.h>
#include <tf/tf.h>
#include <actionlib/server/simple_action_server.h>
#include <eigen_conversions/eigen_msg.h>
#include <tams_ur5_push_execution/PredictPush.h>
#include <tams_ur5_push_execution/PushTrajectory.h>
#include <graph_msgs/GeometryGraph.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <push_planning/PushPlanAction.h>

namespace ob = ompl::base;
namespace oc = ompl::control;

ros::ServiceClient prediction_client_;
ros::Publisher traj_pub_, graph_pub_;

bool isStateValid(const oc::SpaceInformation *si, const ob::State *state)
{
    //    ob::ScopedState<ob::SE2StateSpace>
    // cast the abstract state type to the type we expect
    const auto *se2state = state->as<ob::SE2StateSpace::StateType>();

    // extract the first component of the state and cast it to what we expect
    const auto *pos = se2state->as<ob::RealVectorStateSpace::StateType>(0);

    // extract the second component of the state and cast it to what we expect
    const auto *rot = se2state->as<ob::SO2StateSpace::StateType>(1);

    // check validity of state defined by pos & rot


    // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
    return si->satisfiesBounds(state) && (const void*)rot != (const void*)pos;
}


bool propagateWithPredictor(ros::ServiceClient& predictor,
        const ob::State *start, const oc::Control *control, const double duration, ob::State *result) {

    bool success = true;

    const auto *se2state = start->as<ob::SE2StateSpace::StateType>();
    const double* pos = se2state->as<ob::RealVectorStateSpace::StateType>(0)->values;
    const double yaw = se2state->as<ob::SO2StateSpace::StateType>(1)->value;
    const double* ctrl = control->as<oc::RealVectorControlSpace::ControlType>()->values;

    tams_ur5_push_execution::PredictPush msg;
    msg.request.control.push_back(ctrl[0]);
    msg.request.control.push_back(ctrl[1]);
    msg.request.control.push_back(ctrl[2]);

    // predict push control
    predictor.call(msg);

    if(msg.response.success) {

        geometry_msgs::Pose np = msg.response.next_pose;
        geometry_msgs::Quaternion q = np.orientation;
        Eigen::Quaterniond next_q(q.w, q.x, q.y, q.z);
        double next_yaw = next_q.toRotationMatrix().eulerAngles(0,1,2)[2];

        // create new state
        Eigen::Affine2d next_pos = Eigen::Translation2d(pos[0], pos[1]) 
            * Eigen::Rotation2Dd(yaw) * Eigen::Translation2d(np.position.x, np.position.y);

        // set result state
        result->as<ob::SE2StateSpace::StateType>()->setXY(
                next_pos.translation().x(),
                next_pos.translation().y());
        result->as<ob::SE2StateSpace::StateType>()->setYaw( yaw + next_yaw );

    } else {
        ROS_ERROR_STREAM("Predict Push service call failed!");
        result->as<ob::SE2StateSpace::StateType>()->setXY(pos[0], pos[1]);
        result->as<ob::SE2StateSpace::StateType>()->setYaw(yaw);
        success = false;
    }
    return success;
}

void propagate(const ob::State *start, const oc::Control *control, const double duration, ob::State *result)
{
    propagateWithPredictor(prediction_client_, start, control, duration, result);
    //const auto *se2state = start->as<ob::SE2StateSpace::StateType>();
    //const double* pos = se2state->as<ob::RealVectorStateSpace::StateType>(0)->values;
    //const double yaw = se2state->as<ob::SO2StateSpace::StateType>(1)->value;
    //const double* ctrl = control->as<oc::RealVectorControlSpace::ControlType>()->values;

    //tams_ur5_push_execution::PredictPush msg;
    //msg.request.control.push_back(ctrl[0]);
    //msg.request.control.push_back(ctrl[1]);
    //msg.request.control.push_back(ctrl[2]);

    //// predict push control
    //prediction_client_.call(msg);

    //if(msg.response.success) {

    //    geometry_msgs::Pose np = msg.response.next_pose;
    //    geometry_msgs::Quaternion q = np.orientation;
    //    Eigen::Quaterniond next_q(q.w, q.x, q.y, q.z);
    //    double next_yaw = next_q.toRotationMatrix().eulerAngles(0,1,2)[2];

    //    // create new state
    //    Eigen::Affine2d next_pos = Eigen::Translation2d(pos[0], pos[1]) 
    //        * Eigen::Rotation2Dd(yaw) * Eigen::Translation2d(np.position.x, np.position.y);

    //    // set result state
    //    result->as<ob::SE2StateSpace::StateType>()->setXY(
    //		    next_pos.translation().x(),
    //		    next_pos.translation().y());
    //    result->as<ob::SE2StateSpace::StateType>()->setYaw( yaw + next_yaw );
    //} else {
    //    ROS_ERROR_STREAM("Predict Push service call failed!");
    //    result->as<ob::SE2StateSpace::StateType>()->setXY(pos[0], pos[1]);
    //    result->as<ob::SE2StateSpace::StateType>()->setYaw(yaw);
    //}
}


void pathControlToPushTrajectoryMsg(const ompl::control::PathControl& solution, tams_ur5_push_execution::PushTrajectory& traj_msg) {
    traj_msg.steps = solution.getStateCount();

    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;

    for(int i = 0; i < traj_msg.steps; i++) {
        pose.position.x = solution.getState(i)->as<ob::SE2StateSpace::StateType>()->getX();
        pose.position.y = solution.getState(i)->as<ob::SE2StateSpace::StateType>()->getY();
        double yaw = solution.getState(i)->as<ob::SE2StateSpace::StateType>()->getYaw();
        Eigen::Quaterniond q(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
        tf::quaternionEigenToMsg(q, pose.orientation);
        traj_msg.poses.push_back(pose);
        //traj_msg.pushes.push_back(push);
    }
}

void publishPushTrajectory(const ompl::control::PathControl& solution)
{
    tams_ur5_push_execution::PushTrajectory traj_msg;
    pathControlToPushTrajectoryMsg(solution, traj_msg);
    traj_pub_.publish(traj_msg);
}

void plannerDataToGraphMsg(const ompl::base::PlannerData& data, graph_msgs::GeometryGraph& graph_msg) {
    graph_msg.header.frame_id = "table_top";
    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;

    for(unsigned int i = 0; i < data.numVertices(); i++) {

        // fill node positions
        pose.position.x = data.getVertex(i).getState()->as<ob::SE2StateSpace::StateType>()->getX();
        pose.position.y = data.getVertex(i).getState()->as<ob::SE2StateSpace::StateType>()->getY();
        graph_msg.nodes.push_back(pose.position);

        // copy adjacent edges
        graph_msgs::Edges edges;
        data.getEdges(i, edges.node_ids);

        // copy edge weights
        ompl::base::Cost cost;
        for(unsigned int n : edges.node_ids) {
            if(data.getEdgeWeight(i,n,&cost)) {
                edges.weights.push_back(cost.value());
            }
        }

        // fill edges
        graph_msg.edges.push_back(edges);
    }
}

void publishPlannerData(const ompl::base::PlannerData& data) {
    graph_msgs::GeometryGraph graph_msg;
    plannerDataToGraphMsg(data, graph_msg);
    graph_pub_.publish(graph_msg);
}

void planWithPushControl()
{
    // construct the state space we are planning in
    auto space(std::make_shared<ob::SE2StateSpace>());

    // set the bounds for the R^2 part of SE(2)
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-0.3);
    bounds.setHigh(0.3);

    space->setBounds(bounds);

    // create a control space (obj border, yaw, distance)
    auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 2));

    // we set cbounds to (0.0,1.0) to scale push approaches (border, yaw, distance)
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(0.0);
    cbounds.setHigh(1.0);

    cspace->setBounds(cbounds);

    // define a simple setup class
    oc::SimpleSetup ss(cspace);

    // set the state propagation routine
    ss.setStatePropagator(propagate);

    // set state validity checking for this space
    ss.setStateValidityChecker(
            [&ss](const ob::State *state) { return isStateValid(ss.getSpaceInformation().get(), state); });

    // create a start state
    ob::ScopedState<ob::SE2StateSpace> start(space);
    start->setX(-0.2);
    start->setY(-0.2);
    start->setYaw(0.0);

    // create a  goal state; use the hard way to set the elements
    ob::ScopedState<ob::SE2StateSpace> goal(space);
    (*goal)[0]->as<ob::RealVectorStateSpace::StateType>()->values[0] = 0.2;
    (*goal)[0]->as<ob::RealVectorStateSpace::StateType>()->values[1] = 0.2;
    //(*goal)[1]->as<ob::SO2StateSpace::StateType>()->value = M_PI;
    (*goal)[1]->as<ob::SO2StateSpace::StateType>()->value = 0.0;


    // set the start and goal states
    ss.setStartAndGoalStates(start, goal, 0.05);

    //ss.setPlanner(std::make_shared<oc::RRT>(ss.getSpaceInformation()));
    //ss.getSpaceInformation()->setMinMaxControlDuration(1,100);
    // attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = ss.solve(10.0);

    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        // print the path to screen

        ss.getSolutionPath().printAsMatrix(std::cout);
        publishPushTrajectory(ss.getSolutionPath());

        ompl::base::PlannerData data(ss.getSpaceInformation());
        ss.getPlannerData(data);
        publishPlannerData(data);
    }
    else
        std::cout << "No solution found" << std::endl;
}









namespace push_planning {

    class PushPlannerActionServer
    {
        private:
            actionlib::SimpleActionServer<PushPlanAction> as_;
            ros::ServiceClient predictor_;

        public:
            PushPlannerActionServer(ros::NodeHandle& nh, const std::string& action_name, const std::string& prediction_service_name) :
                as_(nh, action_name, boost::bind(&PushPlannerActionServer::executeCB, this, _1), false)
        {
            if(ros::service::waitForService(prediction_service_name)) {
                predictor_ = nh.serviceClient<tams_ur5_push_execution::PredictPush>(prediction_service_name);
                as_.start();
            }
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

            void setStateToPose(ob::ScopedState<ob::SE2StateSpace>& state, const geometry_msgs::Pose& pose){
                state->setX(pose.position.x);
                state->setY(pose.position.y);
                state->setY(pose.position.z);
                state->setYaw(tf::getYaw(pose.orientation));
            }

            //void propagate(const ob::State *start, const oc::Control *control, const double duration, ob::State *result) {
            //    propagateWithPredictor(predictor_, start, control, duration, result);
            //}

            bool plan(const PushPlanGoalConstPtr& goal, PushPlanResult& result)
            {
                bool success = true;

                // extract goal request
                const std::string& object_id = goal->object_id;
                const geometry_msgs::Pose& start_pose = goal->start_pose;
                const geometry_msgs::Pose& goal_pose = goal->goal_pose;

                // construct the state space we are planning in
                auto space(std::make_shared<ob::SE2StateSpace>());

                // set the bounds for the R^2 part of SE(2)
                ob::RealVectorBounds bounds(2);
                bounds.setLow(-0.3);
                bounds.setHigh(0.3);

                space->setBounds(bounds);

                // create a control space (obj border, yaw, distance)
                auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 2));

                // we set cbounds to (0.0,1.0) to scale push approaches (border, yaw, distance)
                ob::RealVectorBounds cbounds(2);
                cbounds.setLow(0.0);
                cbounds.setHigh(1.0);

                cspace->setBounds(cbounds);

                // define a simple setup class
                oc::SimpleSetup ss(cspace);

                // set the state propagation routine
                ss.setStatePropagator(propagate);
                ss.setStatePropagator( 
                        [&](const ob::State *start, const oc::Control *control, const double duration, ob::State *result)
                        { propagateWithPredictor(predictor_, start, control, duration, result); } );

                // set state validity checking for this space
                ss.setStateValidityChecker(
                        [&ss](const ob::State *state) { return isStateValid(ss.getSpaceInformation().get(), state); });

                // create a start state
                ob::ScopedState<ob::SE2StateSpace> start_state(space);
                setStateToPose(start_state, start_pose);

                // create goal state
                ob::ScopedState<ob::SE2StateSpace> goal_state(space);
                setStateToPose(goal_state, goal_pose);

                // set the start and goal states
                ss.setStartAndGoalStates(start_state, goal_state, 0.05);

                // attempt to solve the problem within ten seconds of planning time
                ob::PlannerStatus solved = ss.solve(10.0);

                if (solved)
                {
                    //ss.getSolutionPath().printAsMatrix(std::cout);
                    pathControlToPushTrajectoryMsg(ss.getSolutionPath(), result.trajectory);

                    ompl::base::PlannerData data(ss.getSpaceInformation());
                    ss.getPlannerData(data);
                    plannerDataToGraphMsg(data, result.planner_data);
                }
                else {
                    result.error_message = "No solution found";
                    success = false;
                }
                return success;
            }
    };
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "push_planner_node");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string predict_push_service = "predict_push_service";

    bool run_test;
    pnh.getParam("run_test", run_test);

    if (!run_test) {
        push_planning::PushPlannerActionServer planner(nh, "/push_plan_action",  predict_push_service);
        ros::spin();
    } else {

        ros::AsyncSpinner spinner(4);
        spinner.start();

        ROS_INFO_STREAM("Waiting for service " << predict_push_service);
        if(ros::service::waitForService(predict_push_service)) {
            ROS_INFO_STREAM("Service: " << predict_push_service << "was found!");
            prediction_client_ = nh.serviceClient<tams_ur5_push_execution::PredictPush>(predict_push_service);
            traj_pub_ = nh.advertise<tams_ur5_push_execution::PushTrajectory>("/push_trajectory", 0);
            graph_pub_ = nh.advertise<graph_msgs::GeometryGraph>("/push_planner_graph", 0);
            planWithPushControl();
        }
    }

    return 0;
}
