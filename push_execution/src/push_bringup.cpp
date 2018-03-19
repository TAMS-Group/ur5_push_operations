#include <math.h>
#include <stdio.h>
#include <boost/variant.hpp>
//#include <ncurses.h>

#include <ros/ros.h>

#include <moveit/robot_state/conversions.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/Mesh.h>
#include <Eigen/Geometry>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <ur5_pusher/pusher.h>
#include <tams_ur5_push_execution/PerformRandomPush.h>
#include <tams_ur5_push_execution/ExplorePushesAction.h>

std::string COMMAND_MAINTENANCE = "maintenance";
std::string COMMAND_GRIPPER_OPEN = "open";
std::string COMMAND_GRIPPER_CLOSE = "close";
std::string COMMAND_PUSHER_ATTACH = "attach";
std::string COMMAND_PUSHER_DETACH = "detach";
std::string COMMAND_DEMO = "demo";
std::string COMMAND_PUSH = "push";
std::string COMMAND_PUSH_NONSTOP = "push nonstop";
std::string COMMAND_SET = "set";
std::string COMMAND_UNSET = "unset";
std::string COMMAND_HELP = "help";
std::string COMMAND_QUIT = "quit";

std::string FLAG_EXECUTE = "execute";

std::vector<double> MAINTENANCE_POSITIONS;
std::vector<std::string> JOINT_NAMES;

class PushExecutionClient {
    public:
        actionlib::SimpleActionClient<tams_ur5_push_execution::ExplorePushesAction> ac_;

        PushExecutionClient(const std::string& action_name) : ac_(action_name)
        {
        }

        bool performRandomPushAction(int samples=100)
        {
           // actionlib::SimpleActionClient<tams_ur5_push_execution::ExplorePushesAction> ac("explore_pushes_action");
            ac_.waitForServer();

            tams_ur5_push_execution::ExplorePushesGoal goal;
            goal.samples = 100;
            ac_.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
            ac_.waitForResult();
            ROS_INFO_STREAM("Explore Pushes finished collecting " << goal.samples << " samples with " << ac_.getResult()->attempts << " attempts.");
            ROS_INFO_STREAM("Total time elapsed: " << ac_.getResult()->elapsed_time.toSec() << " seconds");
            return true;
        }

        bool abortRandomPushAction()
        {
            //TODO: implement abort
            return true;
        }

    private:
        static void doneCb(const actionlib::SimpleClientGoalState& state, 
                const tams_ur5_push_execution::ExplorePushesResultConstPtr& result)
        {
            ROS_INFO_STREAM("Done!");
        }

        static void activeCb()
        {
            ROS_INFO("Goal just went active");
        }

        static void feedbackCb(const tams_ur5_push_execution::ExplorePushesFeedbackConstPtr& feedback)
        {
            //ROS_INFO_STREAM("Successfull sample of push " << feedback->push << " with transform " << feedback->relocation);
            std::cout << "Successfull sample of push " << feedback->push << " with transform " << feedback->relocation << std::endl;
        }
};


class PushBringup
{
	private:
		ur5_pusher::Pusher arm_;
		moveit::planning_interface::MoveGroupInterface gripper_;
		moveit::planning_interface::PlanningSceneInterface psi_;

		ros::NodeHandle nh_;

		bool execute_push_operations_ = false;
        PushExecutionClient pec_;

	public:

		PushBringup() : arm_("arm"), gripper_("gripper"), pec_("explore_pushes_action"){
			createMaintenanceState();

			geometry_msgs::Pose pose;
			pose.position.x = 0.2 - 0.0305;
			pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, -0.5*M_PI, 0.0);

			Eigen::Affine3d transform;
			tf::poseMsgToEigen(pose, transform);

			arm_.setTouchLinks(gripper_.getLinkNames());

			const std::string resource = "package://ur5_push_setup/meshes/pusher_2_aligned_x-binary.stl";
			std::string parent_link = "s_model_tool0";
			arm_.loadPusher(resource, transform, parent_link, "pusher0");
		};

		void createMaintenanceState()
		{
			//TODO: extract values to config
			MAINTENANCE_POSITIONS.push_back(-1.5708);
			MAINTENANCE_POSITIONS.push_back(-3.1413);
			MAINTENANCE_POSITIONS.push_back(0.0);
			MAINTENANCE_POSITIONS.push_back(1.5708);
			MAINTENANCE_POSITIONS.push_back(1.5708);
			MAINTENANCE_POSITIONS.push_back(0.0);

			JOINT_NAMES.push_back("ur5_shoulder_pan_joint");
			JOINT_NAMES.push_back("ur5_shoulder_lift_joint");
			JOINT_NAMES.push_back("ur5_elbow_joint");
			JOINT_NAMES.push_back("ur5_wrist_1_joint");
			JOINT_NAMES.push_back("ur5_wrist_2_joint");
			JOINT_NAMES.push_back("ur5_wrist_3_joint");
		}


		bool moveToMaintenancePose() {
			// move to initial pose
			robot_state::RobotState state = (*arm_.getCurrentState());
			state.setVariablePositions(JOINT_NAMES, MAINTENANCE_POSITIONS);
			arm_.setJointValueTarget(state);
			return bool(arm_.move());
		}

		bool moveToUpPose(double z_val) {
			// move gripper up to mount pusher
			geometry_msgs::Pose pose;
			pose.position.z = z_val;
			pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, 0.0, 0.0);
			arm_.setPoseReferenceFrame("table_top");
			arm_.setPusherPoseTarget(pose);
			return bool(arm_.move());
		}

		bool moveToDownPose(double z_val) {
			// move gripper up to mount pusher
			geometry_msgs::Pose pose;
			pose.position.z = z_val;
			pose.orientation.w = 1.0;
			arm_.setPoseReferenceFrame("table_top");
			arm_.setPusherPoseTarget(pose);
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

		void importMeshFromResource(shape_msgs::Mesh& mesh_msg) {
			std::string resource = "file:///informatik2/students/home/1kayser/ros_demo/src/tams_ur5_push_operations/ur5_push_setup/meshes/pusher_2_aligned-binary.stl";
			Eigen::Vector3d scale(0.001, 0.001, 0.001);
			shapes::Shape* shape = shapes::createMeshFromResource(resource, scale);
			shapes::ShapeMsg shape_msg;
			shapes::constructMsgFromShape(shape, shape_msg);
			mesh_msg = boost::get<shape_msgs::Mesh>(shape_msg);
		}

		moveit_msgs::AttachedCollisionObject getAttachedPusher() {
			// create pusher collision object
			moveit_msgs::CollisionObject pusher;
			pusher.header.frame_id = "s_model_palm";
			pusher.id = "pusher0";
			shape_msgs::Mesh mesh;
			importMeshFromResource(mesh);
			pusher.meshes.push_back(mesh);
			geometry_msgs::Pose pose;
			pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.5*M_PI);
			pose.position.y = 0.2 + 0.05625;
			pusher.mesh_poses.push_back(pose);

			// create AttachedCollisionObject
			moveit_msgs::AttachedCollisionObject attached_pusher;
			attached_pusher.link_name = arm_.getEndEffectorLink();
			attached_pusher.object = pusher;
			attached_pusher.touch_links = gripper_.getLinkNames();
			return attached_pusher;
		}

		void attachPusher() {
			arm_.attachPusher();
		}

		void detachPusher() {
			arm_.detachPusher();
		}

		void setExecute(bool execute) {
			execute_push_operations_ = execute;
		}

		void wait(float seconds) {
			ros::Duration(seconds).sleep();
		}

		bool runDemoMovement(int times=0) {
			// compute and perform 8 movement
			moveToDownPose(0.04);
			arm_.setPoseReferenceFrame("table_top");
			//geometry_msgs::Pose pose;
			geometry_msgs::Pose pose;
			pose.orientation.w = 1.0;
			pose.position.z = 0.04;
			moveit::planning_interface::MoveGroupInterface::Plan plan;
			std::vector<geometry_msgs::Pose> waypoints;
			float radius = 0.05;
			double angle = 0.0;
			for(int i = 0; i < 30; i++) {
				angle = 2 * M_PI * i / 50.0;
				pose.position.x = sin(2 * angle) * radius;
				pose.position.y = -sin(angle) * radius;
				waypoints.push_back(pose);
			}
			float success = arm_.computeCartesianPushPath(waypoints, 0.03, 3, plan.trajectory_);
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

        bool performRandomPushAction(int samples = 100)
        {
            return pec_.performRandomPushAction(samples);
        }

        bool abortRandomPushAction()
        {
            //TODO: implement abort
            return pec_.abortRandomPushAction();
        }
};


void printHelp() {
	std::cout << std::endl;
	std::cout << "========================================" << std::endl;
	std::cout << COMMAND_SET << "/" << COMMAND_UNSET << " <flag> - define bool flags.\nAvailable flags are: " << FLAG_EXECUTE << "\n" << std::endl;
	std::cout << COMMAND_MAINTENANCE << " - move to maintenance pose" << std::endl;
	std::cout << COMMAND_GRIPPER_OPEN << " - open gripper" << std::endl;
	std::cout << COMMAND_GRIPPER_CLOSE << " - close gripper" << std::endl;
	std::cout << COMMAND_PUSHER_ATTACH << " - attach pusher" << std::endl;
	std::cout << COMMAND_PUSHER_DETACH << " - detach pusher" << std::endl;
	std::cout << COMMAND_DEMO << " - run demo" << std::endl;
	std::cout << COMMAND_PUSH << " - perform single push movement" << std::endl;
	std::cout << COMMAND_PUSH_NONSTOP << " - perform nonstop push movements" << std::endl;
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
		} else if (input == COMMAND_PUSHER_ATTACH){
			std::cout << "Attaching pusher" << std::endl;
			pb.attachPusher();
		} else if (input == COMMAND_PUSHER_DETACH){
			std::cout << "Detaching pusher" << std::endl;
			pb.detachPusher();
		} else if (input == COMMAND_DEMO){
			std::cout << "Running demo push movement." << std::endl;
			pb.runDemoMovement(4);
		} else if (input == COMMAND_PUSH){
			std::cout << "Perform random push movement." << std::endl;
			pb.performRandomPushAction(1);
		} else if (input == COMMAND_PUSH_NONSTOP){
			std::cout << "Perform random push movement nonstop!" << std::endl;
            if(pb.performRandomPushAction()) {
                std::cout << "To terminate this operation, press <Enter>" << std::endl;
				std::getline(std::cin, input);
				pb.abortRandomPushAction();
				std::cout << "Push Action terminated by user!" << std::endl;
			} else {
				std::cout << "Server failed to perform push action!" << std::endl;
			}
            /*
			if(pb.performRandomPushNonstop(false)) {
				std::cout << "To terminate this operation, press <Enter>" << std::endl;
				std::getline(std::cin, input);
				pb.performRandomPushNonstop(true);
				std::cout << "Nonstop push operations terminated by user!" << std::endl;
			} else {
				std::cout << "Service failed to perform nonstop push operations!" << std::endl;
			}
            */
		} else if (input == COMMAND_SET + " " + FLAG_EXECUTE){
			pb.setExecute(true);
		} else if (input == COMMAND_UNSET + " " + FLAG_EXECUTE){
			pb.setExecute(true);
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
