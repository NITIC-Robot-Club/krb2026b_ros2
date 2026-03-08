#include "krb2026b_behavior/duck_collector.hpp"

namespace duck_collector {
duck_collector::duck_collector (const rclcpp::NodeOptions &options) : Node ("duck_collector", options) {
    x_offset_            = this->declare_parameter<double> ("x_offset_m", -0.9);
    y_offset_            = this->declare_parameter<double> ("y_offset_m", 0.05);
    filter_gain_         = this->declare_parameter<double> ("filter_gain", 0.5);
    goal_dist_threshold_ = this->declare_parameter<double> ("goal_dist_threshold", 0.1);

    path_pub_         = create_publisher<nav_msgs::msg::Path> ("/planning/path", 10);
    state_result_pub_ = create_publisher<natto_msgs::msg::StateResult> ("state_result", 10);

    current_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped> ("/localization/current_pose", 1, std::bind (&duck_collector::currentPoseCallback, this, std::placeholders::_1));
    map_point_sub_    = create_subscription<geometry_msgs::msg::PointStamped> ("/detection/duck_position", 1, std::bind (&duck_collector::mapPointCallback, this, std::placeholders::_1));
    state_action_sub_ = create_subscription<natto_msgs::msg::StateAction> ("state_action", 10, std::bind (&duck_collector::stateActionCallback, this, std::placeholders::_1));
    goal_reached_sub_ = create_subscription<std_msgs::msg::Bool> ("goal_reached", 1, std::bind (&duck_collector::goalReachedCallback, this, std::placeholders::_1));

    // timer_ = this->create_wall_timer (std::chrono::milliseconds (100), std::bind (&duck_collector::timerCallback, this));

    prev_goal_x_ = 0.0;
    prev_goal_y_ = 0.0;
    path_goal_x_ = 0.0;
    path_goal_y_ = 0.0;

    RCLCPP_INFO (this->get_logger (), "x_offset: %f", x_offset_);
    RCLCPP_INFO (this->get_logger (), "y_offset: %f", y_offset_);
    RCLCPP_INFO (get_logger (), "duck collector Node Started");
}
void duck_collector::stateActionCallback (const natto_msgs::msg::StateAction::SharedPtr msg) {
    if (msg->action_name == "collect_duck") {
        pending_action_msg_ = msg;
        collecting_         = true;
        RCLCPP_INFO (get_logger (), "collect_duck: action received (state_id=%ld), waiting for timer.", msg->state_id);

        if (map_point.point.x == 0.0 && map_point.point.y == 0.0 && map_point.point.z == 0.0) return;
        geometry_msgs::msg::PoseStamped goal_pose;
        goal_pose.header.stamp    = this->now ();
        goal_pose.header.frame_id = "map";

        goal_pose.pose.position.x = map_point.point.x + x_offset_;
        goal_pose.pose.position.y = map_point.point.y + y_offset_;
        goal_pose.pose.position.z = 0.0;

        goal_pose.pose.orientation.x = 0.0;
        goal_pose.pose.orientation.y = 0.0;
        goal_pose.pose.orientation.z = 0.0;
        goal_pose.pose.orientation.w = 1.0;

        planningPath (goal_pose, true);
    }
}

void duck_collector::goalReachedCallback (const std_msgs::msg::Bool::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "goal reached %d, collecting %d", msg->data, collecting_);
    if (!collecting_) return;
    natto_msgs::msg::StateResult result;
    if(pending_action_msg_) {
        result.state_id    = pending_action_msg_->state_id;
    }
    result.action_name = "collect_duck";
    result.success     = msg->data;
    collecting_        = !msg->data;
    state_result_pub_->publish (result);
    // if (msg->data) {
    //     RCLCPP_INFO (get_logger (), "collect_duck: goal reached, publishing success.");
    // }
}

void duck_collector::timerCallback () {
    if (!collecting_) return;
    if (map_point.point.x == 0.0 && map_point.point.y == 0.0 && map_point.point.z == 0.0) return;
    geometry_msgs::msg::PoseStamped goal_pose;
    goal_pose.header.stamp    = this->now ();
    goal_pose.header.frame_id = "map";

    goal_pose.pose.position.x = map_point.point.x + x_offset_;
    goal_pose.pose.position.y = map_point.point.y + y_offset_;
    goal_pose.pose.position.z = 0.0;

    if (prev_goal_x_ != 0.0) goal_pose.pose.position.x = filter_gain_ * goal_pose.pose.position.x + (1.0 - filter_gain_) * prev_goal_x_;
    if (prev_goal_y_ != 0.0) goal_pose.pose.position.y = filter_gain_ * goal_pose.pose.position.y + (1.0 - filter_gain_) * prev_goal_y_;

    prev_goal_x_ = goal_pose.pose.position.x;
    prev_goal_y_ = goal_pose.pose.position.y;

    goal_pose.pose.orientation.x = 0.0;
    goal_pose.pose.orientation.y = 0.0;
    goal_pose.pose.orientation.z = 0.0;
    goal_pose.pose.orientation.w = 1.0;

    planningPath (goal_pose, false);
}
void duck_collector::planningPath (geometry_msgs::msg::PoseStamped goal_pose, bool force_replan) {
    if (current_pose_.pose.position.x == 0.0 && current_pose_.pose.position.y == 0.0 && current_pose_.pose.position.z == 0.0) return;

    double dist_to_prev_goal = std::hypot (goal_pose.pose.position.x - path_goal_x_, goal_pose.pose.position.y - path_goal_y_);
    if (!force_replan && dist_to_prev_goal < goal_dist_threshold_) return;  // 目標が前回の目標と近すぎる場合は再計画しない

    nav_msgs::msg::Path path;
    path.header.stamp    = this->now ();
    path.header.frame_id = "map";

    double current_x   = current_pose_.pose.position.x;
    double current_y   = current_pose_.pose.position.y;
    double current_yaw = quat_to_yaw (current_pose_.pose.orientation);

    double goal_x   = goal_pose.pose.position.x;
    double goal_y   = goal_pose.pose.position.y;
    double goal_yaw = quat_to_yaw (goal_pose.pose.orientation);

    double dx   = goal_x - current_x;
    double dy   = goal_y - current_y;
    double dyaw = goal_yaw - current_yaw;

    while (dyaw > M_PI) dyaw -= 2.0 * M_PI;
    while (dyaw < -M_PI) dyaw += 2.0 * M_PI;

    double total_dist = std::hypot (dx, dy);
    double step       = 0.05;

    int N = std::max (2, static_cast<int> (total_dist / step));

    for (int i = 0; i <= N; i++) {
        double s = static_cast<double> (i) / N;

        // ===== 同時補間 =====
        double x   = current_x + s * dx;
        double y   = current_y + s * dy;
        double yaw = current_yaw + s * dyaw;

        // ===== Pose =====
        geometry_msgs::msg::PoseStamped pose;
        pose.header             = path.header;
        pose.pose.position.x    = x;
        pose.pose.position.y    = y;
        pose.pose.position.z    = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = std::sin (yaw * 0.5);
        pose.pose.orientation.w = std::cos (yaw * 0.5);

        path.poses.push_back (pose);
    }

    path_pub_->publish (path);

    path_goal_x_ = goal_pose.pose.position.x;
    path_goal_y_ = goal_pose.pose.position.y;
}
void duck_collector::currentPoseCallback (const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    current_pose_ = *msg;
}
void duck_collector::mapPointCallback (const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    map_point = *msg;
}
}  // namespace duck_collector

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (duck_collector::duck_collector)