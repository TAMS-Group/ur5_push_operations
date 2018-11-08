#include <ros/ros.h>
#include <push_prediction/push_predictor.h>

namespace push_prediction {

    void PushPredictor::normalizePushInput(const tams_ur5_push_msgs::Push& push, Eigen::VectorXf& input_vec) const
    {
        input_vec.resize(4);
        input_vec(0) =  (0.081 + push.approach.point.x ) / 0.162;
        input_vec(1) = (0.115 + push.approach.point.y ) / 0.23;
        input_vec(2) = std::fmod(tf::getYaw(push.approach.normal), 2 * M_PI) / (2 * M_PI);
        input_vec(3) = std::fmod(push.approach.angle, M_PI) / M_PI;
    }

    void PushPredictor::denormalizePoseOutput(Eigen::VectorXf output_vec, geometry_msgs::Pose& pose) const 
    {
        for (int i = 0; i < output_vec.size(); i++) {
            output_vec(i) = MINV[i] + output_vec(i) * ( MAXV[i] - MINV[i] );
        }
        pose.position.x = output_vec(0);
        pose.position.y = output_vec(1);
        pose.position.z = 0.0;
        tf::quaternionTFToMsg(tf::createQuaternionFromYaw(output_vec(2)), pose.orientation);
    }

    PushPredictor::PushPredictor()
    {
        //network_.load(ros::package::getPath("push_prediction") + "/scripts/keras_model.yaml");
        network_.load(ros::package::getPath("push_prediction") + "/models/model_with_distance.yaml");
    }

    void PushPredictor::setReuseSolutions(bool reuseSolutions) {
        reuseSolutions_ = reuseSolutions;
    }

    bool PushPredictor::predict(const tams_ur5_push_msgs::Push& push, geometry_msgs::Pose& pose) {

        // reuse last solution if request is the same
        if(reuseSolutions_ && pushesEqual(push, last_push)) {
            pose = last_pose;
            return true;
        }
        
        // declare in/out vectors
        Eigen::VectorXf input_vec;
        Eigen::VectorXf output_vec;

        // initialize in vector
        if (network_.hasNormalization()) {
            input_vec.resize(5);
            input_vec(0) = push.approach.point.x;
            input_vec(1) = push.approach.point.y;
            input_vec(2) = tf::getYaw(push.approach.normal);
            input_vec(3) = push.approach.angle;
            input_vec(4) = push.distance;
        } else
            normalizePushInput(push, input_vec);

        // run prediction attempt
        network_.run(input_vec, output_vec);

        // create pose from out vector
        if (network_.hasNormalization()) {
            pose.position.x = output_vec(0);
            pose.position.y = output_vec(1);
            pose.position.z = 0.0;
            tf::quaternionTFToMsg(tf::createQuaternionFromYaw(output_vec(2)), pose.orientation);
        } else
            denormalizePoseOutput(output_vec, pose);

        // persist last request and solution 
        last_push = push;
        last_pose = pose;
        return true;
    }
}

int main(int argc, char** argv)
{
    push_prediction::PushPredictor predictor;
    tams_ur5_push_msgs::Push push;
    push.approach.point.x = -0.081;
    push.approach.point.y = -0.05;
    push.approach.normal.w = 1.0;
    push.approach.angle = 0.0;
    push.distance = 0.01;
    geometry_msgs::Pose pose;
    predictor.predict(push, pose);
    ROS_INFO_STREAM("Push: " << push << ", Pose: " << pose);
    /*
    ros::Time::init();
    ros::Time start_time = ros::Time::now();
    for(int i = 0;i<100000; i++) {
        predictor.predict(push, pose);
    }
    ROS_INFO_STREAM("That took " << (ros::Time::now() - start_time) << "seconds");
    */
    return 0;
}
