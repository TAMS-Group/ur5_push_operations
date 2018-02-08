#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

#include <apriltags_ros/AprilTagDetection.h>
#include <apriltags_ros/AprilTagDetectionArray.h>

#include <XmlRpcValue.h>
#include <XmlRpcException.h>

std::string MARKER_TOPIC = "pushable_objects";
std::string MARKER_NAMESPACE = "pushable_objects";

class ObjectRecognitionNode {
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle ph_;

        ros::Subscriber sub_;

        // TF communication channels
        tf::TransformListener tf_listener_;
        tf::TransformBroadcaster tf_broadcaster_;

        // Marker Publisher
        ros::Publisher marker_pub_;

        // measured object
        tf::Transform object_transform_;
        int object_tag_id_;
        std::string object_frame_prefix_;
        ros::Time detection_time_;

        //ros::Time timeout_;
        bool initialized_;
        bool demo_mode_;
        int demo_object_id_;

        XmlRpc::XmlRpcValue objects_;

    public:
        ObjectRecognitionNode() : initialized_(false) {
            if(!initialized_) {
                ROS_INFO_STREAM("ObjectRecognitionNode running!");
                initialized_ = true;
                nh_ = (*new ros::NodeHandle());
                ph_ = (*new ros::NodeHandle("~"));

                // marker publisher
                marker_pub_ = nh_.advertise<visualization_msgs::Marker>(MARKER_TOPIC, 0);

                // load parameters
                loadParams();

                if (demo_mode_) {
                    // create marker and transform
                    visualization_msgs::Marker marker;
                    if(createObjectMarker(0, marker)) {
                        geometry_msgs::Pose pose;
                        tf::StampedTransform transform = createNormalizedTransform(pose, marker);
                        ros::Rate rate(20);
                        while(ros::ok()) {
                            publishTransformAndMarker(transform, marker);
                            rate.sleep();
                        }
                    } else {
                        ROS_WARN_STREAM("Failed to construct Marker from object configuration!");
                    }
                } else {
                    sub_ = nh_.subscribe("tag_detections", 1, &ObjectRecognitionNode::onDetectAprilTags, this);
                }
            }
        }

        void loadParams(){
            // object descriptions and topic
            ph_.getParam("objects", objects_);
            ph_.param<std::string>("object_frame_prefix_", object_frame_prefix_, "/pushable_object_");

            // demo parameters
            ph_.param<bool>("demo_mode", demo_mode_, false);
            ph_.param<int>("demo_object_id_", demo_object_id_, 0);

            // april tag information
            // TODO: specify object tag mapping
            ph_.param<int>("object_tag_id", object_tag_id_, 11);
            //ph_.param<double>("timeout", timeout_, 10);
        }

        void publishTransformAndMarker(tf::StampedTransform& transform, visualization_msgs::Marker& marker) {
            transform.stamp_ = ros::Time::now();
            tf_broadcaster_.sendTransform(transform);
            marker_pub_.publish(marker);
        }

        void onDetectAprilTags(const apriltags_ros::AprilTagDetectionArray& msg){
            for(apriltags_ros::AprilTagDetection detection : msg.detections) {
                if(detection.id == object_tag_id_) {
                    // create marker
                    visualization_msgs::Marker marker;
                    if(createObjectMarker(object_tag_id_, marker)) {
                        // extract transform from pose
                        detection_time_ = detection.pose.header.stamp;
                        geometry_msgs::PoseStamped pose = detection.pose;
                        tf_listener_.transformPose("/table_top", pose, pose);
                        tf::StampedTransform transform = createNormalizedTransform(detection.pose.pose, marker);
                        publishTransformAndMarker(transform, marker);
                        break;
                    } else {
                        ROS_WARN_STREAM("Failed to create marker for object " << object_tag_id_ << "!");
                    }
                }
            }
        }

        bool createObjectMarker(int object_id, visualization_msgs::Marker& marker) {
            marker.header.frame_id = object_frame_prefix_ + std::to_string(object_id);
            marker.header.stamp = ros::Time();
            marker.ns = MARKER_NAMESPACE;
            marker.id = object_id;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation.w = 1;
            marker.color.a = 0.9;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;

            try {
                XmlRpc::XmlRpcValue& val = objects_[object_id].begin()->second;
                std::string marker_type = val["type"];
                if(marker_type == "CUBE") {
                    marker.type = visualization_msgs::Marker::CUBE;
                } else if (marker_type == "SPHERE") {
                    marker.type = visualization_msgs::Marker::SPHERE;
                } else if (marker_type == "CYLINDER") {
                    marker.type = visualization_msgs::Marker::CYLINDER;
                } else if (marker_type == "MESH") {
                    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
                    if(val["mesh"].valid()) {
                        marker.mesh_resource = static_cast<std::string>(val["mesh"]);
                    } else {
                        ROS_WARN_STREAM("Incomplete definition of pushable object " << object_id << ".");
                        ROS_WARN_STREAM("Object type is MESH but 'mesh_resource' is not specified!");
                        return false;
                    }
                } else {
                    ROS_WARN_STREAM("Found unknown marker type '" << marker_type << "' in definition of object " << object_id);
                    return false;
                }
                marker.scale.x = val["scale"]["x"];
                marker.scale.y = val["scale"]["y"];
                marker.scale.z = val["scale"]["z"];
            } catch (XmlRpc::XmlRpcException& e) {
                ROS_WARN_STREAM("Error extracting values of object " << object_id << " from configuration file!");
                ROS_WARN("%s", e.getMessage().c_str());
                return false;
            }
            return true;
        }

        tf::StampedTransform createNormalizedTransform(geometry_msgs::Pose& pose, visualization_msgs::Marker& marker) {
            //TODO: correct rotation to align with the table!
            pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
            pose.position.z = 0.5 * marker.scale.z;
            tf::Transform transform;
            tf::poseMsgToTF(pose, transform);
            return tf::StampedTransform(transform, detection_time_, "/table_top", object_frame_prefix_ + std::to_string(marker.id));
        }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "object_recognition_node");

    ObjectRecognitionNode obj_rec;
    ROS_INFO_STREAM("ObjectRecognitionNode running!");
    ros::spin();
    return 0;
};
