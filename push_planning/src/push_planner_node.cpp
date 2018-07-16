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
#include <ompl/control/SimpleDirectedControlSampler.h>
#include <ompl/base/StateValidityChecker.h>
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
#include <tams_ur5_push_execution/Push.h>
#include <tams_ur5_push_execution/PredictPush.h>
#include <tams_ur5_push_execution/SteerPush.h>
#include <tams_ur5_push_execution/PushTrajectory.h>
#include <graph_msgs/GeometryGraph.h>
#include <shape_msgs/SolidPrimitive.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <geometry_msgs/Quaternion.h>
#include <push_planning/PushPlanAction.h>
#include <push_prediction/push_predictor.h>
#include <ur5_pusher/push_approach_sampler.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

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

        // TODO: implement control->push conversion
        //const double* ctrl = solution.getControl(i)->as<oc::RealVectorControlSpace::ControlType>()->values;
        //tams_ur5_push_execution::PushApproach push;
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

moveit_msgs::AttachedCollisionObject getAttachedCollisionObject(const double& x, const double& y, const double& z){
    moveit_msgs::AttachedCollisionObject obj;
    obj.link_name = "s_model_tool0";
    obj.object.header.frame_id = "/table_top";
    obj.object.id = "pushable_object";
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.push_back(x);
    primitive.dimensions.push_back(y);
    primitive.dimensions.push_back(z);
    obj.object.primitives.push_back(primitive);
    return obj;
}

void convertStateToPose(const ob::State *state, geometry_msgs::Pose& pose) {
    const auto *se2state = state->as<ob::SE2StateSpace::StateType>();
    pose.position.x = se2state->getX();
    pose.position.y = se2state->getY();
    pose.position.z = 0.0;
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(se2state->getYaw()), pose.orientation);
}

class PushStateValidityChecker : public ob::StateValidityChecker
{
    private:
        const oc::SpaceInformationPtr si_;
        const planning_scene::PlanningScenePtr scene_;

        const double dimX = 0.162;
        const double dimY = 0.23;
        const double dimZ = 0.112;

    public:
        PushStateValidityChecker(const oc::SpaceInformationPtr &si, const planning_scene::PlanningScenePtr scene) : ob::StateValidityChecker(si), si_(si), scene_(scene)
    {

    }

        bool isValid(const ob::State *state) const override
        {
            //    ob::ScopedState<ob::SE2StateSpace>
            // cast the abstract state type to the type we expect
            //const auto *se2state = state->as<ob::SE2StateSpace::StateType>();

            //// extract the first component of the state and cast it to what we expect
            //const auto *pos = se2state->as<ob::RealVectorStateSpace::StateType>(0);

            //// extract the second component of the state and cast it to what we expect
            //const auto *rot = se2state->as<ob::SO2StateSpace::StateType>(1);

            //// check validity of state defined by pos & rot


            //// return a value that is always true but uses the two variables we define, so we avoid compiler warnings
            return si_->satisfiesBounds(state) && !isStateColliding(state);
        }

        bool isStateColliding(const ob::State *state) const
        {
            moveit_msgs::AttachedCollisionObject obj = getAttachedCollisionObject(dimX, dimY, dimZ);
            obj.object.operation = moveit_msgs::CollisionObject::ADD;

            // create object pose
            //const auto *se2state = state->as<ob::SE2StateSpace::StateType>();

            geometry_msgs::Pose pose;
            convertStateToPose(state, pose);
            //pose.position.x = se2state->getX();
            //pose.position.y = se2state->getY();
            pose.position.z = 0.5 * dimZ + 0.001;
            //tf::quaternionTFToMsg(tf::createQuaternionFromYaw(se2state->getYaw()), pose.orientation);

            obj.object.primitive_poses.push_back(pose);
            scene_->processAttachedCollisionObjectMsg(obj);
            return scene_->isStateColliding();
        }
};


class PushStatePropagator : public oc::StatePropagator
{
    private:
        ros::ServiceClient * const steer_predictor_;
        ros::ServiceClient * const push_predictor_;

        const oc::SpaceInformationPtr si_;
        const oc::ControlSamplerPtr cs_;

        push_prediction::PushPredictor* const predictor_;
        ur5_pusher::PushApproachSampler* const push_sampler_;

        const bool canSteer_;

        const double dimX = 0.162;
        const double dimY = 0.23;
        const double dimZ = 0.112;

    public:

        PushStatePropagator(const oc::SpaceInformationPtr &si, ros::ServiceClient& push_predictor, ros::ServiceClient& steer_predictor, bool canSteer=false) 
            : oc::StatePropagator(si), push_predictor_(&push_predictor), steer_predictor_(&steer_predictor), canSteer_(canSteer), predictor_(new push_prediction::PushPredictor()), push_sampler_(new ur5_pusher::PushApproachSampler()), si_(si), cs_(si->allocControlSampler())
        {
            predictor_->setReuseSolutions(true);
        }

        /*
           void propagate(const ob::State *start, const oc::Control *control, const double duration, ob::State *result) const override
           {
           bool success = true;

           const double* ctrl = control->as<oc::RealVectorControlSpace::ControlType>()->values;

        // extract start state
        const auto *se2state = start->as<ob::SE2StateSpace::StateType>();
        const double x = se2state->getX();
        const double y = se2state->getY();
        const double yaw = se2state->getYaw();


        tams_ur5_push_execution::PredictPush msg;
        msg.request.control.push_back(ctrl[0]);
        msg.request.control.push_back(ctrl[1]);
        msg.request.control.push_back(ctrl[2]);

        // predict push control effect
        push_predictor_->call(msg);

        if (msg.response.success) {

        geometry_msgs::Pose np = msg.response.next_pose;
        geometry_msgs::Quaternion q = np.orientation;
        Eigen::Quaterniond next_q(q.w, q.x, q.y, q.z);
        double next_yaw = next_q.toRotationMatrix().eulerAngles(0,1,2)[2];

// create new state
Eigen::Affine2d next_pos = Eigen::Translation2d(x, y) 
         * Eigen::Rotation2Dd(yaw) * Eigen::Translation2d(np.position.x, np.position.y);

// set result state
result->as<ob::SE2StateSpace::StateType>()->setXY(
next_pos.translation().x(),
next_pos.translation().y());
result->as<ob::SE2StateSpace::StateType>()->setYaw( std::fmod(yaw + next_yaw + M_PI , 2 * M_PI) - M_PI);

} else {
ROS_ERROR_STREAM("Predict Push service call failed!");
result->as<ob::SE2StateSpace::StateType>()->setXY(x,y);
result->as<ob::SE2StateSpace::StateType>()->setYaw(yaw);
success = false;
}
}
*/

void propagate(const ob::State *start, const oc::Control *control, const double duration, ob::State *result) const override
{
    const double* ctrl = control->as<oc::RealVectorControlSpace::ControlType>()->values;

    // extract start state
    const auto *se2state = start->as<ob::SE2StateSpace::StateType>();
    const double x = se2state->getX();
    const double y = se2state->getY();
    const double yaw = se2state->getYaw();

    tams_ur5_push_execution::Push push;
    getPushFromControl(control, push);

    // predict push control effect
    geometry_msgs::Pose pose;
    predictor_->predict(push, pose);

    tf::getYaw(pose.orientation);

    geometry_msgs::Quaternion q = pose.orientation;
    Eigen::Quaterniond next_q(q.w, q.x, q.y, q.z);
    double next_yaw = next_q.toRotationMatrix().eulerAngles(0,1,2)[2];

    // create new state
    Eigen::Affine2d next_pos = Eigen::Translation2d(x, y) 
        * Eigen::Rotation2Dd(yaw) * Eigen::Translation2d(pose.position.x, pose.position.y);

    // set result state
    result->as<ob::SE2StateSpace::StateType>()->setXY(
            next_pos.translation().x(),
            next_pos.translation().y());
    result->as<ob::SE2StateSpace::StateType>()->setYaw( std::fmod(yaw + next_yaw + M_PI , 2 * M_PI) - M_PI);
}

void getPushFromControl(const oc::Control *control, tams_ur5_push_execution::Push& push) const
{
    const double* ctrl = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    geometry_msgs::Pose pose = push_sampler_->getPoseFromBoxBorder(ctrl[0], dimX, dimY, dimZ);
	push.approach.point = pose.position;
	push.approach.normal = pose.orientation;
	push.approach.angle = ctrl[1] - 0.5;
	push.distance = ctrl[2] * 0.05;
}

void se2StateToEigen(const ob::State *start, Eigen::Affine2d& pose) const
{
    // extract start state
    const auto *se2state = start->as<ob::SE2StateSpace::StateType>();
    pose.setIdentity();
    pose.translate(Eigen::Vector2d(se2state->getX(), se2state->getX()));
    pose.rotate(Eigen::Rotation2Dd(se2state->getYaw()));
}

double se2Distance(const Eigen::Affine2d& start, const Eigen::Affine2d& goal) const
{
    Eigen::Affine2d diff = start.inverse() * goal;
    return diff.translation().norm() + 0.5 * Eigen::Rotation2Dd(diff.rotation()).angle();
}

/*
 * Implementation of steer_1:
 * Sample random controls (push steps) and propagate as long as the goal distance decreases.
 * If start + n * push reaches a state that is within goal_threshold, return control and duration as step count.
 */
bool steer(const ob::State *start, const ob::State *goal, oc::Control *control, double& duration) const override
{
    Eigen::Affine2d start_pose;
    se2StateToEigen(start, start_pose);

    Eigen::Affine2d goal_pose;
    se2StateToEigen(goal, goal_pose);

    // temp variables
    tams_ur5_push_execution::Push push;
    geometry_msgs::Pose pose;

    const double goal_distance = si_->distance(start, goal);
    const double goal_threshold = 0.05;

    Eigen::Affine2d next_pose, step;

    for(int i = 0; i < 100; i++) {

        // sample control
        cs_->sample(control);
        getPushFromControl(control, push);

        // predict sampled push
        predictor_->predict(push, pose);
        step.setIdentity();
        step.translate(Eigen::Vector2d(pose.position.x, pose.position.y));
        step.rotate(Eigen::Rotation2Dd(tf::getYaw(pose.orientation)));

        // compute push step
        next_pose = start_pose * step;
        double next_distance = se2Distance(next_pose, goal_pose);

        duration = 0.0;
        double min_distance = goal_distance;
        while (next_distance < min_distance) {
            duration = duration + 1.0;
            min_distance = next_distance;
            next_pose = next_pose * step;
            next_distance = se2Distance(next_pose, goal_pose);
        }

        if(duration > 0.0 && min_distance < goal_threshold) {
            return true;
        }
    }
    return false;
}



bool canPropagateBackward()
{
    return false;
}

bool canSteer() const override
{
    return canSteer_;
}

/*
 * Computes control and duration for a given start/goal state pair.
 * The SteerPush service samples random controls and selects the one with the minimal expected
 * goal distance. The distance is computed as defined by SE2StateSpace, as a weighted sum of arclength (0.5)
 * and cartesian distance (1.0).
bool steer(const ob::State *start, const ob::State *to, oc::Control *control, double& duration) const override
{
    tams_ur5_push_execution::SteerPush msg;
    convertStateToPose(start, msg.request.start);
    convertStateToPose(to, msg.request.goal);
    steer_predictor_->call(msg);
    duration = msg.response.duration;
    auto *rcontrol = control->as<oc::RealVectorControlSpace::ControlType>();
    if (msg.response.control.size() == si_->getControlSpace()->getDimension()) {
        for(int i = 0; i < msg.response.control.size(); i++) {
            rcontrol->values[i] = msg.response.control[i];
        }
    }
    return msg.response.success;
}
 */
};


class PushChainControlSampler : public oc::DirectedControlSampler {
    private:
        oc::ControlSamplerPtr cs_;
        unsigned int numControlSamples_;
    public:
        PushChainControlSampler(const oc::SpaceInformation *si, unsigned int k)
            : oc::DirectedControlSampler(si), cs_(si->allocControlSampler()), numControlSamples_(k)
        {
        }

        ~PushChainControlSampler() = default;

        unsigned int getNumControlSamples() const
        {
            return numControlSamples_;
        }

        void setNumControlSamples(unsigned int numSamples)
        {
            numControlSamples_ = numSamples;
        }

        unsigned int sampleTo(oc::Control *control, const ob::State *source,
                ob::State *dest)
        {
            return getBestControl(control, source, dest, nullptr);
        }

        unsigned int sampleTo(oc::Control *control, const oc::Control *previous,
                const ob::State *source, ob::State *dest)
        {
            return getBestControl(control, source, dest, previous);
        }

        unsigned int getBestControl(oc::Control *control, const ob::State *source,
                ob::State *dest, const oc::Control *previous)
        {
            double previous_approach;
            // Sample the first control
            if (previous != nullptr) {
                cs_->sampleNext(control, previous, source);
                previous_approach = previous->as<oc::RealVectorControlSpace::ControlType>()->values[0];
                if (previous_approach > 0.0)
                    control->as<oc::RealVectorControlSpace::ControlType>()->values[0] = previous_approach;
            }
            else
                cs_->sample(control, source);

            const unsigned int minDuration = si_->getMinControlDuration();
            const unsigned int maxDuration = si_->getMaxControlDuration();

            unsigned int steps = cs_->sampleStepCount(minDuration, maxDuration);
            // Propagate the first control, and find how far it is from the target state
            ob::State *bestState = si_->allocState();
            steps = si_->propagateWhileValid(source, control, steps, bestState);

            if (numControlSamples_ > 1)
            {
                oc::Control *tempControl = si_->allocControl();
                ob::State *tempState = si_->allocState();
                double bestDistance = si_->distance(bestState, dest);

                // Sample k-1 more controls, and save the control that gets closest to target
                for (unsigned int i = 1; i < numControlSamples_; ++i)
                {
                    unsigned int sampleSteps = cs_->sampleStepCount(minDuration, maxDuration);
                    if (previous != nullptr) {
                        cs_->sampleNext(tempControl, previous, source);
                        if (previous_approach > 0.0)
                            tempControl->as<oc::RealVectorControlSpace::ControlType>()->values[0] = previous_approach;
                    }
                    else
                        cs_->sample(tempControl, source);

                    sampleSteps = si_->propagateWhileValid(source, tempControl, sampleSteps, tempState);
                    double tempDistance = si_->distance(tempState, dest);
                    if (tempDistance < bestDistance)
                    {
                        si_->copyState(bestState, tempState);
                        si_->copyControl(control, tempControl);
                        bestDistance = tempDistance;
                        steps = sampleSteps;
                    }
                }

                si_->freeState(tempState);
                si_->freeControl(tempControl);
            }

            si_->copyState(dest, bestState);
            si_->freeState(bestState);

            return steps;
        }
};




namespace push_planning {

    class PushPlannerActionServer
    {
        private:
            actionlib::SimpleActionServer<PushPlanAction> as_;
            ros::ServiceClient predictor_;
            ros::ServiceClient steer_predictor_;
            ros::NodeHandle nh_;
            std::string prediction_service_name_;

        public:
            PushPlannerActionServer(ros::NodeHandle& nh, const std::string& action_name, const std::string& prediction_service_name) :
                as_(nh, action_name, boost::bind(&PushPlannerActionServer::executeCB, this, _1), false),
                nh_(nh),
                prediction_service_name_(prediction_service_name)
        {
            if(ros::service::waitForService(prediction_service_name)) {
                predictor_ = nh.serviceClient<tams_ur5_push_execution::PredictPush>(prediction_service_name);
                steer_predictor_ = nh.serviceClient<tams_ur5_push_execution::SteerPush>("steer_push_service");
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
                state->setYaw(tf::getYaw(pose.orientation));
            }

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
                auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 3));

                // we set cbounds to (0.0,1.0) to scale push approaches (border, yaw, distance)
                ob::RealVectorBounds cbounds(3);
                cbounds.setLow(0.0);
                cbounds.setHigh(1.0);

                cspace->setBounds(cbounds);

                // (experimental) set directedcontrolsampler with higher sampling count
                //oc::SpaceInformationPtr si = std::make_shared<oc::SpaceInformation>(cspace->getStateSpace(), cspace);
                //si->setDirectedControlSamplerAllocator([](const oc::SpaceInformation* si){ return std::make_shared<oc::SimpleDirectedControlSampler>(si, 50);});
                //si->setDirectedControlSamplerAllocator([](const oc::SpaceInformation* si){ return std::make_shared<PushChainControlSampler>(si, 10);});

                //oc::SimpleSetup ss(si);

                // define simple setup instance
                oc::SimpleSetup ss(cspace);

                //oc::StatePropagatorPtr push_propagator(std::make_shared<PushStatePropagator>(ss.getSpaceInformation(), predictor_));
                oc::StatePropagatorPtr push_propagator(std::make_shared<PushStatePropagator>(ss.getSpaceInformation(), predictor_, steer_predictor_, true));

                ss.setStatePropagator(push_propagator);

                // set the state propagation routine
                //ss.setStatePropagator( 
                //        [&](const ob::State *start, const oc::Control *control, const double duration, ob::State *result)
                //        { propagateWithPredictor(predictor_, start, control, duration, result); } );


                // load current planning scene
                moveit::planning_interface::PlanningSceneInterface psi;
                std::map<std::string, moveit_msgs::CollisionObject> cobjs = psi.getObjects();


                // apply collision object to planning scene of StateValidityChecker as well
                planning_scene_monitor::PlanningSceneMonitor psm("robot_description");
                psm.startStateMonitor();
                psm.waitForCurrentRobotState(ros::Time::now());
                planning_scene::PlanningScenePtr scene(psm.getPlanningScene());
                for (auto& cobj : cobjs) {
                    scene->processCollisionObjectMsg(cobj.second);
                }

                ob::StateValidityCheckerPtr checker(std::make_shared<PushStateValidityChecker>(ss.getSpaceInformation(), scene));
                ss.setStateValidityChecker(checker);
                //ss.setStateValidityChecker(
                //        [&ss](const ob::State *state) { return isStateValid(ss.getSpaceInformation().get(), state); });

                // create a start state
                ob::ScopedState<ob::SE2StateSpace> start_state(space);
                setStateToPose(start_state, start_pose);

                // create goal state
                ob::ScopedState<ob::SE2StateSpace> goal_state(space);
                setStateToPose(goal_state, goal_pose);

                // set the start and goal states
                ss.setStartAndGoalStates(start_state, goal_state, 0.05);

                ss.setPlanner(std::make_shared<oc::RRT>(ss.getSpaceInformation()));
                ss.getPlanner()->as<oc::RRT>()->setGoalBias(0.5);
                //ss.getPlanner()->as<oc::RRT>()->setGoalBias(0.75);
                ss.getPlanner()->as<oc::RRT>()->setIntermediateStates(true);
                ss.getSpaceInformation()->setMinMaxControlDuration(1,100);
                ss.getSpaceInformation()->setPropagationStepSize(1.0);
                // attempt to solve the problem within ten seconds of planning time
                ob::PlannerStatus solved = ss.solve(300.0);

                if (solved)
                {
                    //ss.getSolutionPath().printAsMatrix(std::cout);
                    pathControlToPushTrajectoryMsg(ss.getSolutionPath(), result.trajectory);

                    ob::PlannerData data(ss.getSpaceInformation());
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

    bool spawn_cobj;
    pnh.param("spawn_collision_object_test", spawn_cobj, false);
    if(spawn_cobj) {
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

    std::string predict_push_service = "predict_push_service";

    push_planning::PushPlannerActionServer planner(nh, "/push_plan_action",  predict_push_service);
    ros::spin();
    return 0;
}
