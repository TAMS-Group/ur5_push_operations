#include <math.h>

#include <ros/ros.h>

#include <moveit/robot_state/conversions.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>


int main(int argc, char** argv) {
	geometry_msgs::Quaternion down = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.5*M_PI, 0.0);
	geometry_msgs::Quaternion up = tf::createQuaternionMsgFromRollPitchYaw(0.0, -0.5*M_PI, 0.0);
	ros::init(argc,argv, "push_bringup_node");
	ros::AsyncSpinner spinner(4);
	spinner.start();
	
	//initialize movegroup
	moveit::planning_interface::MoveGroupInterface arm("arm");

	// move to initial pose
	arm.setNamedTarget("pour_default");
	arm.move();

	// move gripper up to mount pusher
	geometry_msgs::Pose pose;
	pose.position.z = 0.5;
	pose.orientation = up;
	arm.setPoseReferenceFrame("table_top");
	arm.setPoseTarget(pose);
	arm.move();

	ros::Duration(3).sleep();

	// move pusher down to table
	pose.position.z = 0.04;
	pose.orientation = down;
	arm.setPoseTarget(pose);
	arm.move();

	// compute and perform 8 movement
	moveit::planning_interface::MoveGroupInterface::Plan plan;
	std::vector<geometry_msgs::Pose> waypoints;
	float radius = 0.05;
	double angle = 0.0;
	for(int i = 0; i < 50; i++) {
		angle = 2 * M_PI * i / 50.0;
		pose.position.x = sin(2 * angle) * radius;
		pose.position.y = -sin(angle) * radius;
		waypoints.push_back(pose);
	}
	float success = arm.computeCartesianPath(waypoints, 0.03, 3, plan.trajectory_);
	if(success==1.0) {
		while(ros::ok()) {
			arm.execute(plan);
		}
	} else {
		ROS_INFO_STREAM("Failed 8 trajectory with percentage " << success);
	}
	return 0;
}
