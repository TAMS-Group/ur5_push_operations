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

namespace{
    void interpolateTransforms(const tf::Transform& t1, const tf::Transform& t2, double fraction, tf::Transform& t_out) {
        t_out.setOrigin( t1.getOrigin()*(1-fraction) + t2.getOrigin()*fraction );
        t_out.setRotation( t1.getRotation().slerp(t2.getRotation(), fraction) );
    }
}

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
        tf::Transform new_transform_;
        tf::Transform transform_;
        visualization_msgs::Marker marker_;
        int object_id_;
        int object_tag_id_;
        std::string object_frame_prefix_;
        ros::Time detection_time_;

        //ros::Time timeout_;
        bool initialized_;
        bool demo_mode_;
        int demo_object_id_;

        XmlRpc::XmlRpcValue objects_;

        bool knows_transform_ = false;
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
                        transform_ = createNormalizedTransform(pose, marker);
                        ros::Rate rate(20);
                        while(ros::ok()) {
                            detection_time_ = ros::Time(0);
                            publishTransformAndMarker(transform_, marker);
                            rate.sleep();
                        }
                    } else {
                        ROS_WARN_STREAM("Failed to construct Marker from object configuration!");
                    }
                } else {
                    sub_ = nh_.subscribe("tag_detections", 1, &ObjectRecognitionNode::onDetectAprilTags, this);
                    ros::Rate rate(10);
                    while(ros::ok()) {
                        if(knows_transform_) {
                            interpolateTransforms(transform_, new_transform_, 0.5, transform_);
                            marker_.header.stamp = ros::Time(0);
                            publishTransformAndMarker(transform_, marker_);
                        } else {
                            ROS_WARN_STREAM_THROTTLE(1, "Cannot find transform of object '" << marker_.header.frame_id << "'");
                        }
                        rate.sleep();
                    }
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
            ph_.param<int>("object_id", object_id_, 11);
            object_tag_id_ = objects_[object_id_].begin()->second["tag_id"];
            ROS_INFO_STREAM("Looking for object " << object_id_ << " with tag " << object_tag_id_);
            //ph_.param<double>("timeout", timeout_, 10);
        }

        void publishTransformAndMarker(tf::Transform& transform, visualization_msgs::Marker& marker) {
            tf_broadcaster_.sendTransform(tf::StampedTransform(transform, detection_time_, "/table_top", object_frame_prefix_ + std::to_string(marker.id)));
            marker_pub_.publish(marker);
        }

        void onDetectAprilTags(const apriltags_ros::AprilTagDetectionArray& msg){
            for(apriltags_ros::AprilTagDetection detection : msg.detections) {
                ROS_ERROR_STREAM("Detected april tag nr: " << detection.id);
                if(detection.id == object_tag_id_) {
                    // create marker
                    if(createObjectMarker(object_id_, marker_)) {
                        // extract transform from pose
                        detection_time_ = detection.pose.header.stamp;
                        geometry_msgs::PoseStamped pose = detection.pose;
                        pose.header.stamp = ros::Time(0);
                        try{
                            tf_listener_.transformPose("/table_top", pose, pose);
                            new_transform_ = createNormalizedTransform(pose.pose, marker_);
                            if(!knows_transform_) {
                                transform_ = new_transform_;
                                knows_transform_ = true;
                            }
                        }
                        catch(...){
                            ROS_WARN_STREAM_THROTTLE(10, "Waiting for table_top->" << pose.header.frame_id << " transform");
                        }
                    } else {
                        ROS_WARN_STREAM("Failed to create marker for object " << object_id_ << "!");
                    }
                }
            }
        }

        bool createObjectMarker(int object_id, visualization_msgs::Marker& marker) {
            marker.header.frame_id = object_frame_prefix_ + std::to_string(object_id);
            marker.header.stamp = ros::Time();
            marker.lifetime = ros::Duration(1);
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

        tf::Transform createNormalizedTransform(geometry_msgs::Pose& pose, visualization_msgs::Marker& marker) {
            float yaw = 0.0;
            if(!demo_mode_) {
                //get only yaw part of detected pose
                yaw = tf::getYaw(pose.orientation) + 0.5*M_PI;
                try {
                    XmlRpc::XmlRpcValue& val = objects_[marker.id].begin()->second;
                    double x_off = val["tag_offset"]["x"];
                    double y_off = val["tag_offset"]["y"];
                    pose.position.x += x_off;
                    pose.position.y += y_off;
                } catch (XmlRpc::XmlRpcException& e) {
                    ROS_WARN_STREAM("Error extracting values of object " << marker.id << " from configuration file!");
                    ROS_WARN("%s", e.getMessage().c_str());
                }
            }
            pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
            pose.position.z = 0.5 * marker.scale.z;
            tf::Transform transform;
            tf::poseMsgToTF(pose, transform);
            //return tf::StampedTransform(transform, detection_time_, "/table_top", object_frame_prefix_ + std::to_string(marker.id));
            return transform;
        }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "object_recognition_node");
    ros::AsyncSpinner spinner(4);

    spinner.start();

    ObjectRecognitionNode obj_rec;
    return 0;
};
