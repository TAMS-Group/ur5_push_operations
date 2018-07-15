#include <ros/package.h>
#include <tams_ur5_push_execution/Push.h>
#include <geometry_msgs/Pose.h>
#include <push_prediction/neural_network.h>
#include <tf/transform_datatypes.h>

//#include <eigen3/Eigen/Geometry>
//#include <eigen3/Eigen/Core>
#include <Eigen/Dense>

const std::vector<double> MAXV={0.03178563295947549, 0.029346353059715446, 0.26358129169260636};
const std::vector<double> MINV = {-0.04123226483869708, -0.031217403199005074, -0.22898143957295725};

namespace push_prediction {
    class PushPredictor {
        private:
            NeuralNetwork network_;
            bool reuseSolutions_ = false;
            
            tams_ur5_push_execution::Push last_push;
            geometry_msgs::Pose last_pose;


        protected:
            void normalizePushInput(const tams_ur5_push_execution::Push& push, Eigen::VectorXf& input_vec) const;

            void denormalizePoseOutput(Eigen::VectorXf output_vec, geometry_msgs::Pose& pose) const;
        public:
            PushPredictor();

            void setReuseSolutions(bool reuseSolutions);

            bool pushesEqual(const tams_ur5_push_execution::Push& first, const tams_ur5_push_execution::Push& second) {
                return first.approach.point.x == second.approach.point.x 
                    && first.approach.point.y == second.approach.point.y
                    && first.approach.normal.w == second.approach.normal.w
                    && first.approach.normal.x == second.approach.normal.x
                    && first.approach.normal.y == second.approach.normal.y
                    && first.approach.normal.z == second.approach.normal.z
                    && first.approach.angle == second.approach.angle
                    && first.distance == second.distance;
            }

            bool predict(const tams_ur5_push_execution::Push& push, geometry_msgs::Pose& pose);
    };
}
