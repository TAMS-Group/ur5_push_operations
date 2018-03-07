#include <algorithm>

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <tf/transform_listener.h>

#include <eigen_conversions/eigen_msg.h>

#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/Mesh.h>


#include <visualization_msgs/Marker.h>

#pragma once

namespace ur5_pusher
{

class Pusher : public moveit::planning_interface::MoveGroupInterface
{

	private:
		moveit::planning_interface::PlanningSceneInterface psi_;

		std::string mesh_resource_;
		float mesh_scale_;
		Eigen::Affine3d tip_transform_;
		moveit_msgs::AttachedCollisionObject pusher_object_;
		std::string parent_link_;
		std::string pusher_id_;
		std::vector<std::string> touch_links_;

		bool pusher_attached_ = false;
		bool knows_pusher_ = false;

		bool loadPusher();
		moveit_msgs::AttachedCollisionObject getPusherObjectMsg(const shape_msgs::Mesh& mesh_msg, const geometry_msgs::Pose& pose_msg) const;
		void importMeshFromResource(const std::string& resource, shape_msgs::Mesh& mesh_msg, float scale) const;

		bool getSingleAttachedObject(moveit_msgs::AttachedCollisionObject& object);

		bool hasSingleAttachedObject();

	public:
		Pusher(const std::string& group_name);

		Pusher(const std::string& group_name, const std::string& resource, const Eigen::Affine3d& transform, const std::string& parent_link, const std::string& pusher_id);

		void setPusherMeshResource(const std::string& resource, float scale=0.001);

		void setPusherTipTransform(const Eigen::Affine3d transform);

		void setPusherParentLink(const std::string& parent_link);

		void setPusherId(const std::string& pusher_id);

		void setTouchLinks(const std::vector<std::string>& touch_links);

		bool loadPusher(const std::string& resource, const Eigen::Affine3d& transform, const std::string& parent_link, const std::string& pusher_id);

		bool loadFromAttachedObject();

		bool isPusherAttached();

		bool knowsPusher() const;

		bool attachPusher();

		bool detachPusher();

		bool setPusherPoseTarget(const geometry_msgs::Pose& pose);

		bool setPusherPoseTarget(const Eigen::Affine3d& pose);

		bool setPusherJointValueTarget(const geometry_msgs::PoseStamped& pose_stamped);

		bool setPusherJointValueTarget(const geometry_msgs::Pose& pose);

		bool setPusherJointValueTarget(const Eigen::Affine3d& pose);

		double computeCartesianPushPath(std::vector<geometry_msgs::Pose>& waypoints, double eef_step, double jump_threshold, moveit_msgs::RobotTrajectory& trajectory);
};
}
