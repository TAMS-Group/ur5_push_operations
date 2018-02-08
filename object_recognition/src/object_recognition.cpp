#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <apriltags_ros/AprilTagDetection.h>
#include <apriltags_ros/AprilTagDetectionArray.h>

class ObjectRecognitionNode {
    private:
        ros::Subscriber sub_;

        // TF communication channels
        tf::TransformListener tf_listener_;
        tf::TransformBroadcaster tf_broadcaster_;

        // measured object
        tf::Transform object_transform_;
        int object_tag_id_;
        std::string object_frame_id_;
        ros::Time detection_time_;

        //ros::Time timeout_;

        bool initialized_;
        bool dummy_mode_;

    public:
        ObjectRecognitionNode() : initialized_(false) {
            ros::NodeHandle node;
            ros::NodeHandle private_handle("~");
            private_handle.param<bool>("dummy_mode", dummy_mode_, false);
            private_handle.param<int>("object_tag_id", object_tag_id_, 11);
            private_handle.param<std::string>("object_frame_id", object_frame_id_, "/object_frame_" + std::to_string(object_tag_id_));
            //private_handle.param<double>("timeout", timeout_, 10);
            if (!dummy_mode_) {
                sub_ = node.subscribe("tag_detections", 1, &ObjectRecognitionNode::callback, this);
            } else {
                geometry_msgs::Pose pose;
                pose.orientation.w = 1;
                pose.position.z = 0.05;
                ros::Rate rate(20);
                while(ros::ok()) {
                    tf::poseMsgToTF(pose, object_transform_);
                    tf_broadcaster_.sendTransform(tf::StampedTransform(object_transform_, ros::Time::now(), "/table_top", object_frame_id_));
                    rate.sleep();
                }
            }
        }

        void callback(const apriltags_ros::AprilTagDetectionArray& msg){
            for(apriltags_ros::AprilTagDetection detection : msg.detections) {
                if(detection.id == object_tag_id_) {
                    tf::poseMsgToTF(detection.pose.pose, object_transform_);
                    detection_time_ = detection.pose.header.stamp;
                    if(!initialized_) {
                        ROS_INFO("object recognition is running");
                        initialized_ = true;
                    }

                    //Send transform with actually measured detection time
                    tf_broadcaster_.sendTransform(tf::StampedTransform(object_transform_, detection_time_, "/world", object_frame_id_));
                    break;
                }
            }
        }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "object_recognition_node");
    ObjectRecognitionNode obj_rec;
    ros::spin();
    return 0;
};
