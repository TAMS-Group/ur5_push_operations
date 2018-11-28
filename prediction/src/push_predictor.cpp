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


#include <ros/ros.h>
#include <cmath>
#include <push_prediction/push_predictor.h>

namespace push_prediction {

    void PushPredictor::normalizePushInput(const tams_ur5_push_msgs::Push& push, Eigen::VectorXf& input_vec) const
    {
        input_vec.resize(4);
        input_vec(0) =  (0.081 + push.approach.point.x ) / 0.162;
        input_vec(1) = (0.115 + push.approach.point.y ) / 0.23;
        input_vec(2) = std::fmod(tf::getYaw(push.approach.normal), 2 * M_PI) / (2 * M_PI);
        input_vec(3) = std::fmod(push.approach.angle, M_PI) / M_PI;
    }

    void PushPredictor::denormalizePoseOutput(Eigen::VectorXf output_vec, geometry_msgs::Pose& pose) const 
    {
        for (int i = 0; i < output_vec.size(); i++) {
            output_vec(i) = MINV[i] + output_vec(i) * ( MAXV[i] - MINV[i] );
        }
        pose.position.x = output_vec(0);
        pose.position.y = output_vec(1);
        pose.position.z = 0.0;
        tf::quaternionTFToMsg(tf::createQuaternionFromYaw(output_vec(2)), pose.orientation);
    }

    PushPredictor::PushPredictor(const std::string& model_file)
    {
        network_.load(model_file);
    }

    PushPredictor::PushPredictor()
	    : PushPredictor(ros::package::getPath("tams_ur5_push_prediction") + "/models/model_with_distance.yaml"){}


    void PushPredictor::setReuseSolutions(bool reuseSolutions) {
        reuseSolutions_ = reuseSolutions;
    }

    bool PushPredictor::predict(const tams_ur5_push_msgs::Push& push, geometry_msgs::Pose& pose) {

        // reuse last solution if request is the same
        if(reuseSolutions_ && pushesEqual(push, last_push)) {
            pose = last_pose;
            return true;
        }
        
        // declare in/out vectors
        Eigen::VectorXf input_vec;
        Eigen::VectorXf output_vec;

        // initialize in vector
        if (network_.hasNormalization()) {
            input_vec.resize(5);
            input_vec(0) = push.approach.point.x;
            input_vec(1) = push.approach.point.y;
            double yaw = tf::getYaw(push.approach.normal);
            yaw += M_PI / 2;
            yaw = std::fmod(yaw, 1.5 * M_PI);
            input_vec(2) = yaw - M_PI / 2;
            input_vec(3) = push.approach.angle;
            input_vec(4) = push.distance;
        } else
            normalizePushInput(push, input_vec);

        // run prediction attempt
        network_.run(input_vec, output_vec);

        // create pose from out vector
        if (network_.hasNormalization()) {
            pose.position.x = output_vec(0);
            pose.position.y = output_vec(1);
            pose.position.z = 0.0;
            tf::quaternionTFToMsg(tf::createQuaternionFromYaw(output_vec(2)), pose.orientation);
        } else
            denormalizePoseOutput(output_vec, pose);

        // persist last request and solution 
        last_push = push;
        last_pose = pose;
        return true;
    }
}

int main(int argc, char** argv)
{
    push_prediction::PushPredictor predictor;
    tams_ur5_push_msgs::Push push;
    push.approach.point.x = -0.081;
    push.approach.point.y = -0.05;
    push.approach.normal.w = 1.0;
    push.approach.angle = 0.0;
    push.distance = 0.01;
    geometry_msgs::Pose pose;
    predictor.predict(push, pose);
    ROS_INFO_STREAM("Push: " << push << ", Pose: " << pose);
    /*
    ros::Time::init();
    ros::Time start_time = ros::Time::now();
    for(int i = 0;i<100000; i++) {
        predictor.predict(push, pose);
    }
    ROS_INFO_STREAM("That took " << (ros::Time::now() - start_time) << "seconds");
    */
    return 0;
}
