#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/CollisionObject.h>

namespace push_execution
{
	class Pusher
	{
		public:

			moveit_msgs::CollisionObject& obj_;

			std::string& frame_id_;

			const moveit::planning_interface::MoveGroupInterface& group_;


			Pusher(const moveit::planning_interface::MoveGroupInterface& group,
					moveit_msgs::CollisionObject& obj,
					std::string& frame_id) : obj_(obj), frame_id_(frame_id), group_(group){};

			void setTargetObject(moveit_msgs::CollisionObject& obj)
			{
				obj_ = obj;
			}

			void setPoseReferenceFrame(std::string& frame_id)
			{
				frame_id_ = frame_id;
			}

			bool executeRandomPush()
			{
				geometry_msgs::Pose push_pose = sampleRandomPushPose(obj_);
			}

		private:

			geometry_msgs::Pose sampleRandomPushPose(moveit_msgs::CollisionObject& obj)
			{
				geometry_msgs::Pose push_pose;
				push_pose.position = sampleRandomContactPoint(obj);

				push_pose.orientation = sampleRandomPushAngle(obj, push_pose.position);

				return push_pose;
			}

			geometry_msgs::Point sampleRandomContactPoint(moveit_msgs::CollisionObject& obj)
			{
				geometry_msgs::Point contact_point;
				//Sample contact point from obj shape
				return contact_point;
			}

			geometry_msgs::Quaternion sampleRandomPushAngle(moveit_msgs::CollisionObject& obj, geometry_msgs::Point contact_point)
			{
				geometry_msgs::Quaternion push_angle;
				//Sample push angle from contact point
				return push_angle;
			}

			bool executePush(geometry_msgs::Pose push_pose, double push_dist=0.02, double approach_dist=0.05, double retrace_dist=0.03)
			{
				//compute cartesian path from pre push, push and retrace
				return true;
			}
	};
}
