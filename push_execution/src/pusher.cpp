#include <ur5_pusher/pusher.h>

namespace ur5_pusher
{

    bool Pusher::loadPusher()
    { 
        if(!pusher_attached_) {
            shape_msgs::Mesh mesh_msg;
            importMeshFromResource(mesh_resource_, mesh_msg, mesh_scale_);
            geometry_msgs::Pose pose;
            tf::poseEigenToMsg(tip_transform_, pose);
            pusher_object_ = getPusherObjectMsg(mesh_msg, pose);
            knows_pusher_ = true;
        } else {
            ROS_WARN("There is still a pusher attached! Please detach it first before loading a new pusher model!");
        }
    }

    moveit_msgs::AttachedCollisionObject Pusher::getPusherObjectMsg(const shape_msgs::Mesh& mesh_msg, const geometry_msgs::Pose& pose_msg) const {
        // create pusher collision object
        moveit_msgs::CollisionObject pusher;
        pusher.header.frame_id = parent_link_;
        pusher.id = pusher_id_;
        pusher.meshes.push_back(mesh_msg);
        pusher.mesh_poses.push_back(pose_msg);

        moveit_msgs::AttachedCollisionObject attached_pusher;
        attached_pusher.link_name = parent_link_;
        attached_pusher.object = pusher;
        attached_pusher.touch_links = touch_links_;
        return attached_pusher;
    }

    void Pusher::importMeshFromResource(const std::string& resource, shape_msgs::Mesh& mesh_msg, float scale) const {
        Eigen::Vector3d scaling(scale, scale, scale);
        shapes::Shape* shape = shapes::createMeshFromResource(resource, scaling);
        shapes::ShapeMsg shape_msg;
        shapes::constructMsgFromShape(shape, shape_msg);
        mesh_msg = boost::get<shape_msgs::Mesh>(shape_msg);
    }

    Pusher::Pusher(const std::string& group_name) : moveit::planning_interface::MoveGroupInterface(group_name) {}

    Pusher::Pusher(const std::string& group_name, const std::string& resource, const Eigen::Affine3d& transform, const std::string& parent_link, const std::string& pusher_id) : moveit::planning_interface::MoveGroupInterface(group_name) {
        loadPusher(resource, transform, parent_link, pusher_id);
    }

    void Pusher::setPusherMeshResource(const std::string& resource, float scale)
    {
        mesh_resource_ = resource;
        mesh_scale_ = scale;
    }

    void Pusher::setPusherTipTransform(const Eigen::Affine3d transform)
    {
        tip_transform_ = transform;
    }

    void Pusher::setPusherParentLink(const std::string& parent_link)
    {
        parent_link_ = parent_link;
    }

    void Pusher::setPusherId(const std::string& pusher_id)
    {
        pusher_id_ = pusher_id;
    }

    void Pusher::setTouchLinks(const std::vector<std::string>& touch_links)
    {
        touch_links_ = touch_links;
    }

    bool Pusher::loadPusher(const std::string& resource, const Eigen::Affine3d& transform, const std::string& parent_link, const std::string& pusher_id)
    {
        setPusherMeshResource(resource);
        setPusherTipTransform(transform);
        setPusherParentLink(parent_link);
        setPusherId(pusher_id);
        return loadPusher();
    }

    bool Pusher::isPusherAttached() const
    {
        return pusher_attached_;
    }

    bool Pusher::knowsPusher() const
    {
        return knows_pusher_; 
    }

    bool Pusher::attachPusher()
    {
        if(knows_pusher_) {
            pusher_object_.object.operation = moveit_msgs::CollisionObject::ADD;
            psi_.applyAttachedCollisionObject(pusher_object_);
            pusher_attached_ = true;
            return true;
        } 

        ROS_WARN("Could not attach pusher since it is not sufficiently defined!");
        return false;
    }

    bool Pusher::detachPusher()
    {
        if(pusher_attached_) {
            pusher_object_.object.operation = moveit_msgs::CollisionObject::REMOVE;
            psi_.applyAttachedCollisionObject(pusher_object_);
            pusher_attached_ = false;
            return true;
        } 

        ROS_WARN("No pusher attached! Nothing to do here.");
        return false;
    }

    bool Pusher::setPusherPoseTarget(const geometry_msgs::Pose& pose) {
        Eigen::Affine3d pose_affine;
        tf::poseMsgToEigen(pose, pose_affine);
        return setPusherPoseTarget(pose_affine);
    }

    bool Pusher::setPusherPoseTarget(const Eigen::Affine3d& pose) {
        return setPoseTarget(pose * tip_transform_.inverse());
    }

    bool Pusher::setPusherJointValueTarget(const geometry_msgs::PoseStamped& pose_stamped) {
        Eigen::Affine3d pose_affine;
        tf::poseMsgToEigen(pose_stamped.pose, pose_affine);
        return setPusherJointValueTarget(pose_affine, pose_stamped.header.frame_id);
    }

    bool Pusher::setPusherJointValueTarget(const Eigen::Affine3d& pose, const std::string& frame) {
        return setJointValueTarget(pose * tip_transform_.inverse(), frame);
    }

    double Pusher::computeCartesianPushPath(std::vector<geometry_msgs::Pose>& waypoints, double eef_step, double jump_threshold, moveit_msgs::RobotTrajectory& trajectory) {
        std::vector<geometry_msgs::Pose> transformed_waypoints;
        Eigen::Affine3d affine;
        for(geometry_msgs::Pose wp : waypoints) {
            tf::poseMsgToEigen(wp, affine);
            geometry_msgs::Pose transformed_wp;
            tf::poseEigenToMsg(affine * tip_transform_.inverse(), transformed_wp);
            transformed_waypoints.push_back(transformed_wp);
        }
        return computeCartesianPath(transformed_waypoints, eef_step, jump_threshold, trajectory);
    }

    //TODO: Implement getCurrentPusherPose
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "push_execution_node");
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ur5_pusher::Pusher pusher("arm");
    return 0;
}


