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
#include <cmath>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>

double linearDistance(const geometry_msgs::Pose& start, const geometry_msgs::Pose& goal) {
	using namespace std;
	double x_diff = start.position.x - goal.position.x;
	double y_diff = start.position.y - goal.position.y;
	return sqrt(pow(x_diff,2) + pow(y_diff,2));
}

double yawDistance(const geometry_msgs::Pose& start, const geometry_msgs::Pose& goal) {
	using namespace std;
	tf::Quaternion q_start, q_goal;
	tf::quaternionMsgToTF(start.orientation, q_start);
	tf::quaternionMsgToTF(goal.orientation, q_goal);
	double yaw = fmod(abs(tf::getYaw(q_start.inverse() * q_goal)), 2 * M_PI);
	return yaw > M_PI ? 2 * M_PI - yaw : yaw;
}

double se2Distance(const geometry_msgs::Pose& start, const geometry_msgs::Pose& goal) {
	return linearDistance(start, goal) + 0.5 * yawDistance(start, goal);
}


