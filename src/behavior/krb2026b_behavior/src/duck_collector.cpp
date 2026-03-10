#include "krb2026b_behavior/duck_collector.hpp"

namespace duck_collector {
duck_collector::duck_collector (const rclcpp::NodeOptions &options) : Node ("duck_collector", options) {
    x_offset_            = this->declare_parameter<double> ("x_offset_m", -0.9);
    y_offset_            = this->declare_parameter<double> ("y_offset_m", 0.05);
    filter_gain_         = this->declare_parameter<double> ("filter_gain", 0.5);
    goal_dist_threshold_ = this->declare_parameter<double> ("goal_dist_threshold", 0.1);

    goal_pose_pub_    = create_publisher<geometry_msgs::msg::PoseStamped> ("goal_pose", 10);
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

        if (x_buffer_.empty () || y_buffer_.empty ()) return;
        std::vector<double> x_vec (x_buffer_.begin (), x_buffer_.end ());
        std::vector<double> y_vec (y_buffer_.begin (), y_buffer_.end ());
        auto                mid_x = x_vec.begin () + x_vec.size () / 2;
        auto                mid_y = y_vec.begin () + y_vec.size () / 2;
        std::nth_element (x_vec.begin (), mid_x, x_vec.end ());
        std::nth_element (y_vec.begin (), mid_y, y_vec.end ());

        goal_pose.pose.position.x = *mid_x + x_offset_;
        goal_pose.pose.position.y = *mid_y + y_offset_;
        goal_pose.pose.position.z = 0.0;

        goal_pose.pose.orientation.x = 0.0;
        goal_pose.pose.orientation.y = 0.0;
        goal_pose.pose.orientation.z = 0.0;
        goal_pose.pose.orientation.w = 1.0;

        goal_pose_pub_->publish(goal_pose);
    } else {
        collecting_ = false;
    }
}

void duck_collector::goalReachedCallback (const std_msgs::msg::Bool::SharedPtr msg) {
    if (!collecting_) return;
    natto_msgs::msg::StateResult result;
    if (pending_action_msg_) {
        result.state_id = pending_action_msg_->state_id;
    }
    result.action_name = "collect_duck";
    result.success     = msg->data;
    collecting_        = !msg->data;
    state_result_pub_->publish (result);
}
void duck_collector::currentPoseCallback (const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    current_pose_ = *msg;
}
void duck_collector::mapPointCallback (const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    map_point = *msg;

    x_buffer_.push_back (map_point.point.x);
    y_buffer_.push_back (map_point.point.y);
    if (x_buffer_.size () > 10) x_buffer_.pop_front ();
    if (y_buffer_.size () > 10) y_buffer_.pop_front ();
}
}  // namespace duck_collector

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (duck_collector::duck_collector)