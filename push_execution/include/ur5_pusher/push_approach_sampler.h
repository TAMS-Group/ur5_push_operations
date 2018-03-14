#include <tf/transform_listener.h>

#include <eigen_conversions/eigen_msg.h>

#include <visualization_msgs/Marker.h>

#include <tams_ur5_push_execution/Push.h>
#include <tams_ur5_push_execution/PushApproach.h>

#pragma once

std::random_device rd;
std::mt19937 gen{rd()};

namespace ur5_pusher
{

class PushApproachSampler
{
	private:
		visualization_msgs::Marker marker_;
		const float safety_range_;
		const float emergency_range_;

		const float min_table_distance_;
		const float tip_length_;

		std::string reference_frame_;

		tf::TransformListener tf_listener_;

		bool sampleApproachPoseAndAngle(geometry_msgs::Pose& pose, double& angle, int attempts=100);
		geometry_msgs::Pose sampleRandomPoseFromBox(double dim_x, double dim_y, double dim_z);

		float sampleRandomPushAngle(float range=0.5);

	public:
		PushApproachSampler(const float safety_range, const float emergency_range, const float min_table_distance, const float tip_length);
		void setMarker(const visualization_msgs::Marker& marker);
		void setReferenceFrame(const std::string& reference_frame);

		bool sampleRandomPushApproach(tams_ur5_push_execution::PushApproach& approach);
};
}
