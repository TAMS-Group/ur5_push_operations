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

#include <queue>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <tams_ur5_push_msgs/FTDump.h>
#include <robotiq_ft_sensor/ft_sensor.h>

namespace push_msgs = tams_ur5_push_msgs;

namespace tams_ur5_push_sensing {

  class FTDumpService {

    //struct ImageDumpTask {
    //    std::string filename;
    //    sensor_msgs::ImageConstPtr image;
    //};

    private:
      ros::ServiceServer service_;

      rosbag::Bag bag_;
      std::string dir_name_;
      std::string ft_topic_;
      bool dump_ft_data_ = false;

      bool onRequestFTDump(push_msgs::FTDump::Request& req, push_msgs::FTDump::Response& res)
      {
        ROS_INFO_STREAM("Request force torque sensor dump: " << req.filename);

        if (req.enabled)
        {
          if (dump_ft_data_)
          {
            ROS_ERROR("Recording of multiple force-torque rosbags at the same time is not supported");
            return false;
          }
          bag_.open(dir_name_ + "/" + req.filename + ".bag", rosbag::bagmode::Write);
          dump_ft_data_ = true;
        }
        else
        {
          dump_ft_data_ = false;
          bag_.close();
        }
        return true;
      }

      void sensorCallback(const robotiq_ft_sensor::ft_sensor& ft_msg)
      {
        if (dump_ft_data_)
        {
          bag_.write(ft_topic_, ros::Time::now(), ft_msg);

          //if (bag_.isOpen())
          //  bag_.write(ft_topic_, ros::Time::now(), ft_msg);
          //else
          //  ROS_ERROR("Attempting to write ft-sensor data to non-open rosbag");
        }
      }

    public:
      FTDumpService(const std::string& ft_topic, const std::string& dir_name) : dir_name_(dir_name),
      ft_topic_(ft_topic)
    {
      ros::NodeHandle nh;
      service_ = nh.advertiseService("/ft_dump_service", &FTDumpService::onRequestFTDump, this);
      ros::Subscriber sub = nh.subscribe(ft_topic_, 1, &FTDumpService::sensorCallback, this);
      ros::waitForShutdown();
    }
  };
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ft_dump_service");
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::NodeHandle nh;
  std::string push_results_directory;
  if(nh.getParam("push_result_directory", push_results_directory)) {
    tams_ur5_push_sensing::FTDumpService ftds("/robotiq_ft_sensor", push_results_directory + "/ft_data");
  } else {
    ROS_ERROR("Failed to retrieve push results directory!");
  }

  return 0;
}
