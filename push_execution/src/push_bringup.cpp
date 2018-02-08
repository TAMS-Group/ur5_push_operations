#include <math.h>
#include <stdio.h>
//#include <ncurses.h>

#include <ros/ros.h>

#include <moveit/robot_state/conversions.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>

std::string COMMAND_MAINTENANCE = "m";
std::string COMMAND_GRIPPER_OPEN = "o";
std::string COMMAND_GRIPPER_CLOSE = "c";
std::string COMMAND_DEMO = "r";
std::string COMMAND_HELP = "h";
std::string COMMAND_QUIT = "q";


class PushBringup
{
	public:
	geometry_msgs::Quaternion orientation_down_ = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.5*M_PI, 0.0);
	geometry_msgs::Quaternion orientation_up_ = tf::createQuaternionMsgFromRollPitchYaw(0.0, -0.5*M_PI, 0.0);
	moveit::planning_interface::MoveGroupInterface arm_;
	moveit::planning_interface::MoveGroupInterface gripper_;


		PushBringup() : arm_("arm"), gripper_("gripper"){
		};

		bool moveToMaintenancePose() {
			// move to initial pose
			arm_.setNamedTarget("push_maintenance");
			return bool(arm_.move());
		}

		bool moveToUpPose(double z_val) {
			// move gripper up to mount pusher
			geometry_msgs::Pose pose;
			pose.position.z = z_val;
			pose.orientation = orientation_up_;
			arm_.setPoseReferenceFrame("table_top");
			arm_.setPoseTarget(pose);
			return bool(arm_.move());
		}

		bool moveToDownPose(double z_val) {
			// move gripper up to mount pusher
			geometry_msgs::Pose pose;
			pose.position.z = z_val;
			pose.orientation = orientation_down_;
			arm_.setPoseReferenceFrame("table_top");
			arm_.setPoseTarget(pose);
			return bool(arm_.move());
		}

		bool openGripper() {
			gripper_.setNamedTarget("open");
			gripper_.move();
		}

		bool closeGripper() {
			gripper_.setNamedTarget("closed");
			gripper_.move();
		}

		void wait(float seconds) {
			ros::Duration(seconds).sleep();
		}

		bool runDemoMovement(int times=0) {
			// compute and perform 8 movement
			moveToDownPose(0.04);
			geometry_msgs::Pose pose = arm_.getCurrentPose().pose;
			arm_.setPoseReferenceFrame("table_top");
			//geometry_msgs::Pose pose;
			pose.position.z = 0.04;
			//pose.orientation = orientation_down_;
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
			float success = arm_.computeCartesianPath(waypoints, 0.03, 3, plan.trajectory_);
			if(success==1.0) {
				int count = 0;
				// TODO: add keyboard callback
				while((times==0 || count++ <= times)) {
					if (!arm_.execute(plan)) 
						return false;
				}
				return true;
			} else {
				ROS_INFO_STREAM("\nFailed 8 trajectory with percentage " << success);
				return false;
			}
		}
};

void printHelp() {
	std::cout << std::endl;
	std::cout << "========================================" << std::endl;
	std::cout << COMMAND_MAINTENANCE << " - move to maintenance pose" << std::endl;
	std::cout << COMMAND_GRIPPER_OPEN << " - open gripper" << std::endl;
	std::cout << COMMAND_GRIPPER_CLOSE << " - close gripper" << std::endl;
	std::cout << COMMAND_DEMO << " - run demo" << std::endl;
	std::cout << COMMAND_HELP << " - show help screen" << std::endl;
	std::cout << COMMAND_QUIT << " - quit program" << std::endl;
}


int main(int argc, char** argv) {
	geometry_msgs::Quaternion down = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.5*M_PI, 0.0);
	geometry_msgs::Quaternion up = tf::createQuaternionMsgFromRollPitchYaw(0.0, -0.5*M_PI, 0.0);
	ros::init(argc,argv, "push_bringup_node", ros::init_options::NoRosout);
	ros::AsyncSpinner spinner(4);
	spinner.start();
	PushBringup pb;

	std::cout << "========================================" << std::endl;
	std::cout << std::endl << "Launching Push operator bringup" << std::endl;

	printHelp();
	std::cout << std::endl << "Waiting for input:";

	std::string input;

	while(ros::ok()) {
		std::cout << std::endl << ">";

		// Display prompt and read input (NB: input must be freed after use)...
		std::getline(std::cin, input);
		if(input == COMMAND_MAINTENANCE) {
			std::cout << "Moving to maintenance pose" << std::endl;
			pb.moveToMaintenancePose();
		} else if (input == COMMAND_GRIPPER_OPEN){
			std::cout << "Opening gripper" << std::endl;
			pb.openGripper();
		} else if (input == COMMAND_GRIPPER_CLOSE){
			std::cout << "Closing gripper" << std::endl;
			pb.closeGripper();
		} else if (input == COMMAND_DEMO){
			std::cout << "Running demo push movement." << std::endl;
			pb.runDemoMovement(4);
		} else if (input == COMMAND_HELP){
			printHelp();
		} else if (input == COMMAND_QUIT){
			std::cout << "Bye!" << std::endl;
			break;
		} else {
			std::cout << "Unkown command: '" << input  << "'" << std::endl;
			printHelp();
		}
	}
	return 0;
}
