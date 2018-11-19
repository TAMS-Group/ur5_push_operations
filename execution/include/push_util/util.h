#pragma once
#include <cmath>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>

double se2distance(const geometry_msgs::Pose& start, const geometry_msgs::Pose& goal) {
	using namespace std;
	double x_diff = start.position.x - goal.position.x;
	double y_diff = start.position.y - goal.position.y;
	double translation = sqrt(pow(x_diff,2) + pow(y_diff,2));

	tf::Quaternion q_start;
	tf::Quaternion q_goal;
	tf::quaternionMsgToTF(start.orientation, q_start);
	tf::quaternionMsgToTF(goal.orientation, q_goal);
	double yaw = fmod(tf::getYaw(q_start.inverse() * q_goal), 2*M_PI);
	yaw = abs(min(yaw, 2*M_PI - yaw));
	return translation + 0.5 * yaw;
}
