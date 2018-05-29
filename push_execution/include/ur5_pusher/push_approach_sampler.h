#include <tf/transform_listener.h>

#include <eigen_conversions/eigen_msg.h>

#include <visualization_msgs/Marker.h>

#include <tams_ur5_push_execution/Push.h>
#include <tams_ur5_push_execution/PushApproach.h>

#pragma once

std::random_device rd;
std::mt19937 gen{rd()};
std::uniform_real_distribution<double> unif_dist_{0.0,1.0};

namespace ur5_pusher
{

const float MIN_TABLE_DISTANCE = 0.02;
const float TIP_LENGTH = 0.08;

// Range to restrict the object on the table
const float SAFETY_RANGE = 0.03; // Outside of this range the object is pushed towards the center
const float EMERGENCY_RANGE = 0.3; // Outside of this range the experiment is aborted

class PushApproachSampler
{
	private:

		visualization_msgs::Marker marker_;
		float safety_range_;
		float emergency_range_;

		float min_table_distance_;
		float approach_distance_;
		float tip_length_;
		float tip_radius_;

		std::string reference_frame_;

		tf::TransformListener tf_listener_;

		bool sampleApproachPoseAndAngle(geometry_msgs::Pose& pose, double& angle, int attempts=100);
		geometry_msgs::Pose sampleRandomPoseFromBox(double dim_x, double dim_y, double dim_z);

		double sampleRandomPushAngle(double range=0.5);
		double sampleRandomPushDistance(double min=0.005, double max=0.03);

	public:
		PushApproachSampler();
		void setMarker(const visualization_msgs::Marker& marker);
		void setReferenceFrame(const std::string& reference_frame);

		bool sampleRandomPush(tams_ur5_push_execution::Push& push);
		bool sampleRandomPushApproach(tams_ur5_push_execution::PushApproach& approach);

};
}
