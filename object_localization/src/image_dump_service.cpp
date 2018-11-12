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
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <tams_ur5_push_msgs/ImageDump.h>

namespace push_msgs = tams_ur5_push_msgs;

namespace tams_ur5_push_object_localization {

    class ImageDumpService {

        struct ImageDumpTask {
            std::string filename;
            sensor_msgs::ImageConstPtr image;
        };

        private:
        ros::ServiceServer service_;

        std::string dir_name_;

        std::queue<ImageDumpTask> image_dump_tasks_;
        sensor_msgs::ImageConstPtr image_ = NULL;

        bool onRequestImageDump(push_msgs::ImageDump::Request& req, push_msgs::ImageDump::Response& res)
        {
            ROS_INFO_STREAM("Request image dump: " << req.filename);
            const sensor_msgs::ImageConstPtr image(image_);

            if(image!=NULL) {
                ImageDumpTask t;
                t.filename = req.filename;
                t.image = image;
                image_dump_tasks_.push(t);
            }
        }

        void imageCallback(const sensor_msgs::ImageConstPtr& image)
        {
            image_ = image;
        }

        void dumpImage(const ImageDumpTask& task) {
            cv_bridge::CvImageConstPtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvShare(task.image, sensor_msgs::image_encodings::BGR8);
                std::vector<int> compression_params;
                compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
                compression_params.push_back(9);
                cv::imwrite(dir_name_ + "/" + task.filename + ".png", cv_ptr->image);
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
            catch (std::runtime_error& e)
            {
                ROS_ERROR("Exception converting image to PNG format: %s", e.what());
                return;
            }
        }

        public:
        ImageDumpService(const std::string& image_topic, const std::string& dir_name) : dir_name_(dir_name)
        {
            ros::NodeHandle nh;
            service_ = nh.advertiseService("/image_dump_service", &ImageDumpService::onRequestImageDump, this);

            image_transport::ImageTransport it(nh);
            image_transport::Subscriber sub = it.subscribe(image_topic, 1, boost::bind(&ImageDumpService::imageCallback, this, _1));

            ros::Rate r(2);
            while(ros::ok()) {
                if(!image_dump_tasks_.empty()) {
                    dumpImage(image_dump_tasks_.front());
                    image_dump_tasks_.pop();
                }
                if(image_dump_tasks_.size() > 5) {
                    ROS_WARN_STREAM_THROTTLE(10, "Image dump service is unable to handle request volume. Number of pending requests: " << image_dump_tasks_.size());
                }
                r.sleep();
            }
        }
    };
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_dump_service");
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::NodeHandle nh;
    std::string push_results_directory;
    if(nh.getParam("push_result_directory", push_results_directory)) {
	    tams_ur5_push_object_localization::ImageDumpService ids("/kinect2/qhd/image_color", push_results_directory + "/images");
    } else {
	    ROS_ERROR("Failed to retrieve push results directory!");
    }

    return 0;
}

