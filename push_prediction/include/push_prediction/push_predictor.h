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


#pragma once

#include <ros/package.h>
#include <tams_ur5_push_msgs/Push.h>
#include <geometry_msgs/Pose.h>
#include <push_prediction/neural_network.h>
#include <tf/transform_datatypes.h>

//#include <eigen3/Eigen/Geometry>
//#include <eigen3/Eigen/Core>
#include <Eigen/Dense>

const std::vector<double> MAXV={0.03178563295947549, 0.029346353059715446, 0.26358129169260636};
const std::vector<double> MINV = {-0.04123226483869708, -0.031217403199005074, -0.22898143957295725};

namespace push_prediction {
    class PushPredictor {
        private:
            NeuralNetwork network_;
            bool reuseSolutions_ = false;
            
            tams_ur5_push_msgs::Push last_push;
            geometry_msgs::Pose last_pose;


        protected:
            void normalizePushInput(const tams_ur5_push_msgs::Push& push, Eigen::VectorXf& input_vec) const;

            void denormalizePoseOutput(Eigen::VectorXf output_vec, geometry_msgs::Pose& pose) const;
        public:
            PushPredictor();

            void setReuseSolutions(bool reuseSolutions);

            bool pushesEqual(const tams_ur5_push_msgs::Push& first, const tams_ur5_push_msgs::Push& second) {
                return first.approach.point.x == second.approach.point.x 
                    && first.approach.point.y == second.approach.point.y
                    && first.approach.normal.w == second.approach.normal.w
                    && first.approach.normal.x == second.approach.normal.x
                    && first.approach.normal.y == second.approach.normal.y
                    && first.approach.normal.z == second.approach.normal.z
                    && first.approach.angle == second.approach.angle
                    && first.distance == second.distance;
            }

            bool predict(const tams_ur5_push_msgs::Push& push, geometry_msgs::Pose& pose);
    };
}
