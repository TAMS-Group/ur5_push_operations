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

#include  <cli/operations.h>

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
  std::cout << COMMAND_PUSH_BACK << " - move object back to table center" << std::endl;
  std::cout << COMMAND_SAVE_TARGET << " - save current object position as push target" << std::endl;
  std::cout << COMMAND_PUSH_TARGET << " - push object to push target" << std::endl;
  std::cout << COMMAND_HELP << " - show help screen" << std::endl;
  std::cout << COMMAND_QUIT << " - quit program" << std::endl;
}

int main(int argc, char** argv) {
  geometry_msgs::Quaternion down = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.5*M_PI, 0.0);
  geometry_msgs::Quaternion up = tf::createQuaternionMsgFromRollPitchYaw(0.0, -0.5*M_PI, 0.0);
  ros::init(argc,argv, "push_bringup_node", ros::init_options::NoRosout);
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::NodeHandle nh;
  std::string push_result_dir;
  if(!nh.getParam("push_result_directory", push_result_dir))
  {
    ROS_ERROR_STREAM("Unable to start bringup without 'push_result_directory' set. Make sure to launch ur5_push_bringup first!");
    return 1;
  }

  bool dump_feedback;
  nh.param("dump_feedback", dump_feedback, true);
  PushOperations op(push_result_dir, dump_feedback);

  std::cout << "========================================" << std::endl;
  std::cout << std::endl << "Launching Push operator bringup" << std::endl;

  printHelp();
  std::cout << std::endl << "Waiting for input:";

  std::string input;

  while(ros::ok() && !std::cin.eof()) {
    std::cout << std::endl << "> ";

    // Display prompt and read input (NB: input must be freed after use)...
    std::getline(std::cin, input);
    if(input == COMMAND_MAINTENANCE) {
      std::cout << "Moving to maintenance pose" << std::endl;
      op.moveToMaintenancePose();
    } else if (input == COMMAND_GRIPPER_OPEN){
      std::cout << "Opening gripper" << std::endl;
      op.openGripper();
    } else if (input == COMMAND_GRIPPER_CLOSE){
      std::cout << "Closing gripper" << std::endl;
      op.closeGripper();
    } else if (input == COMMAND_PUSHER_ATTACH){
      std::cout << "Attaching pusher" << std::endl;
      op.attachPusher();
    } else if (input == COMMAND_PUSHER_DETACH){
      std::cout << "Detaching pusher" << std::endl;
      op.detachPusher();
    } else if (input == COMMAND_DEMO){
      std::cout << "Running demo push movement." << std::endl;
      op.runDemoMovement(4);
    } else if (input == COMMAND_PUSH){
      std::cout << "Perform random push movement." << std::endl;
      op.performRandomPushAction(1);
    } else if (input == COMMAND_PUSH_NONSTOP){
      std::cout << "Perform random push movement nonstop!" << std::endl;
      if(op.performRandomPushAction(0)) {
        std::cout << "To terminate this operation, press <Enter>" << std::endl;
        std::getline(std::cin, input);
        op.abortAction();
        std::cout << "Push Action terminated by user!" << std::endl;
      } else {
        std::cout << "Server failed to perform push action!" << std::endl;
      }
    } else if (input == COMMAND_PUSH_BACK){
      std::cout << "Trying to push object back to the table center!" << std::endl;
      if(op.pushObjectBack()) {
        std::cout << "To terminate this operation, press <Enter>" << std::endl;
        std::getline(std::cin, input);
        op.abortAction();
        std::cout << "Push Action terminated by user!" << std::endl;
      } else {
        std::cout << "Server failed to perform push action!" << std::endl;
      }
    } else if (input == COMMAND_SAVE_TARGET){
      if(op.savePushTarget()) {
        std::cout << "Goal target saved. Push Object there with '" << COMMAND_PUSH_TARGET << "'" << std::endl;
      } else {
        std::cout << "Failed to save push target due to unknown transform." << std::endl;
      }
    } else if (input == COMMAND_PUSH_TARGET){
      std::cout << "Attempt to push object to target." << std::endl;
      if(op.pushToTarget()) {
        std::cout << "To terminate this operation, press <Enter>" << std::endl;
        std::getline(std::cin, input);
        op.abortAction();
        std::cout << "Push Action terminated by user!" << std::endl;
      } else {
        std::cout << "Server failed to perform push action!" << std::endl;
      }
    } else if (input == COMMAND_POINT){
      std::cout << "Point at pushable object." << std::endl;
      op.pointAtBox();
    } else if (input == COMMAND_SET + " " + FLAG_EXECUTE){
      op.setExecute(true);
    } else if (input == COMMAND_UNSET + " " + FLAG_EXECUTE){
      op.setExecute(true);
    } else if (input == COMMAND_QUIT || std::cin.eof()){
      std::cout << "Bye!" << std::endl;
      break;
    } else {
      std::cout << "Unkown command: '" << input  << "'" << std::endl;
      printHelp();
    }
  }
  return 0;
}
