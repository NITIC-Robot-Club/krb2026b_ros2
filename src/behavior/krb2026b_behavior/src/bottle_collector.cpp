#include "krb2026b_behavior/bottle_collector.hpp"

namespace bottle_collector {

bottle_collector::bottle_collector (const rclcpp::NodeOptions &node_options) : Node ("bottle_collector", node_options) {
    path_step_m_       = this->declare_parameter<double> ("path_step_m", 0.05);
    offset_normal_m_   = this->declare_parameter<double> ("offset_normal_m", 0.325);
    offset_large_m_    = this->declare_parameter<double> ("offset_large_m", 0.42);
    y_thresh_m_        = this->declare_parameter<double> ("y_thresh_m", 0.02);
    yaw_thresh_rad_    = this->declare_parameter<double> ("yaw_thresh_deg", 3.0) * M_PI / 180.0;
    replan_position_thresh_m_ = this->declare_parameter<double> ("replan_position_thresh_m", 0.03);
    replan_yaw_thresh_rad_    = this->declare_parameter<double> ("replan_yaw_thresh_deg", 2.0) * M_PI / 180.0;
    double frequency   = this->declare_parameter<double> ("frequency", 100.0);

    tf_buffer_   = std::make_shared<tf2_ros::Buffer> (this->get_clock ());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener> (*tf_buffer_);

    state_result_publisher_ = this->create_publisher<natto_msgs::msg::StateResult> ("state_result", 10);
    path_publisher_         = this->create_publisher<nav_msgs::msg::Path> ("path", 10);
    goal_pose_publisher_    = this->create_publisher<geometry_msgs::msg::PoseStamped> ("collect_goal_pose", 10);

    state_action_subscriber_ = this->create_subscription<natto_msgs::msg::StateAction> ("state_action", 10, std::bind (&bottle_collector::state_action_callback, this, std::placeholders::_1));
    bottle_pairs_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseArray> ("bottle_pairs", 10, std::bind (&bottle_collector::bottle_pairs_callback, this, std::placeholders::_1));
    goal_reached_subscriber_ = this->create_subscription<std_msgs::msg::Bool> ("goal_reached", 10, std::bind (&bottle_collector::goal_reached_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer (std::chrono::duration<double> (1.0 / frequency), std::bind (&bottle_collector::timer_callback, this));

    RCLCPP_INFO (this->get_logger (), "bottle_collector node has been initialized.");
    RCLCPP_INFO (this->get_logger (), "path_step_m: %f", path_step_m_);
    RCLCPP_INFO (this->get_logger (), "offset_normal_m: %f", offset_normal_m_);
    RCLCPP_INFO (this->get_logger (), "offset_large_m: %f", offset_large_m_);
    RCLCPP_INFO (this->get_logger (), "y_thresh_m: %f", y_thresh_m_);
    RCLCPP_INFO (this->get_logger (), "yaw_thresh_deg: %f", yaw_thresh_rad_ * 180.0 / M_PI);
    RCLCPP_INFO (this->get_logger (), "replan_position_thresh_m: %f", replan_position_thresh_m_);
    RCLCPP_INFO (this->get_logger (), "replan_yaw_thresh_deg: %f", replan_yaw_thresh_rad_ * 180.0 / M_PI);
    RCLCPP_INFO (this->get_logger (), "frequency: %f Hz", frequency);
}

void bottle_collector::state_action_callback (const natto_msgs::msg::StateAction::SharedPtr msg) {
    if (msg->action_name == "collect_bottle") {
        pending_action_msg_ = msg;
        collecting_         = true;
        has_last_target_    = false;
        RCLCPP_INFO (this->get_logger (), "collect_bottle: action received (state_id=%ld), waiting for timer.", msg->state_id);
    }
}

void bottle_collector::timer_callback () {
    if (!collecting_) return;
    collect_bottle (pending_action_msg_);
}

void bottle_collector::bottle_pairs_callback (const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    if (collecting_) {
        latest_bottle_pairs_ = msg;
    }
}

void bottle_collector::goal_reached_callback (const std_msgs::msg::Bool::SharedPtr msg) {
    if (!collecting_) return;
    if (msg->data) {
        natto_msgs::msg::StateResult result;
        result.state_id    = pending_action_msg_->state_id;
        result.action_name = "collect_bottle";
        result.success     = true;
        collecting_        = false;
        has_last_target_   = false;
        state_result_publisher_->publish (result);
        RCLCPP_INFO (this->get_logger (), "collect_bottle: goal reached, publishing success.");
    }
}

void bottle_collector::collect_bottle (const natto_msgs::msg::StateAction::SharedPtr msg) {
    natto_msgs::msg::StateResult result;
    result.state_id    = msg->state_id;
    result.action_name = "collect_bottle";

    if (!latest_bottle_pairs_ || latest_bottle_pairs_->poses.empty ()) {
        RCLCPP_WARN (this->get_logger (), "collect_bottle: no bottle pairs available.");
        result.success = false;
        state_result_publisher_->publish (result);
        return;
    }

    geometry_msgs::msg::TransformStamped tf_base_to_map;
    try {
        tf_base_to_map = tf_buffer_->lookupTransform ("map", "base_link", rclcpp::Time (0));
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN (this->get_logger (), "collect_bottle: TF lookup failed: %s", ex.what ());
        result.success = false;
        state_result_publisher_->publish (result);
        return;
    }

    const auto &target   = latest_bottle_pairs_->poses[0];
    double      tx_map   = target.position.x;
    double      ty_map   = target.position.y;
    double      tyaw_map = quat_to_yaw (target.orientation);

    if (has_last_target_) {
        double target_shift_m = std::hypot (tx_map - last_target_x_map_, ty_map - last_target_y_map_);
        double target_yaw_diff = std::fabs (normalize_angle (tyaw_map - last_target_yaw_map_));
        if (target_shift_m < replan_position_thresh_m_ && target_yaw_diff < replan_yaw_thresh_rad_) {
            RCLCPP_DEBUG_THROTTLE (this->get_logger (), *this->get_clock (), 1000, "collect_bottle: target change is small (d=%.3f m, dyaw=%.2f deg), skipping republish.", target_shift_m,
                                   target_yaw_diff * 180.0 / M_PI);
            return;
        }
    }

    auto [tx_base, ty_base] = inverse_transform_point (tx_map, ty_map, tf_base_to_map);
    double robot_yaw_map    = quat_to_yaw (tf_base_to_map.transform.rotation);
    double tyaw_base        = tyaw_map - robot_yaw_map;
    while (tyaw_base > M_PI) tyaw_base -= 2.0 * M_PI;
    while (tyaw_base < -M_PI) tyaw_base += 2.0 * M_PI;

    double approach_yaw = tyaw_base + M_PI;
    while (approach_yaw > M_PI) approach_yaw -= 2.0 * M_PI;
    while (approach_yaw < -M_PI) approach_yaw += 2.0 * M_PI;

    RCLCPP_INFO (this->get_logger (), "collect_bottle: target in base_link: tx=%.3f  ty=%.3f  tyaw=%.1f deg  approach_yaw=%.1f deg", tx_base, ty_base, tyaw_base * 180.0 / M_PI, approach_yaw * 180.0 / M_PI);

    bool need_large_offset = (std::fabs (ty_base) > y_thresh_m_) || (std::fabs (approach_yaw) > yaw_thresh_rad_);
    double offset_m = need_large_offset ? offset_large_m_ : offset_normal_m_;

    double gx_base = tx_base + offset_m * std::cos (approach_yaw);
    double gy_base = ty_base + offset_m * std::sin (approach_yaw);

    RCLCPP_INFO (this->get_logger (), "collect_bottle: using %s offset=%.3f m", need_large_offset ? "large" : "normal", offset_m);

    auto path_msg = generate_path (gx_base, gy_base, approach_yaw, tf_base_to_map);
    path_msg.header.stamp    = this->now ();
    path_msg.header.frame_id = "map";
    path_publisher_->publish (path_msg);

    auto [goal_x_map, goal_y_map] = transform_point (gx_base, gy_base, tf_base_to_map);
    double goal_yaw_map           = approach_yaw + robot_yaw_map;

    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id    = "map";
    goal.header.stamp       = this->now ();
    goal.pose.position.x    = goal_x_map;
    goal.pose.position.y    = goal_y_map;
    goal.pose.orientation.z = std::sin (goal_yaw_map * 0.5);
    goal.pose.orientation.w = std::cos (goal_yaw_map * 0.5);
    goal_pose_publisher_->publish (goal);

    last_target_x_map_   = tx_map;
    last_target_y_map_   = ty_map;
    last_target_yaw_map_ = tyaw_map;
    has_last_target_     = true;
}

nav_msgs::msg::Path bottle_collector::generate_path (double gx_base, double gy_base, double gyaw_base, const geometry_msgs::msg::TransformStamped &tf_base_to_map) {
    nav_msgs::msg::Path path;

    double       robot_yaw_map = quat_to_yaw (tf_base_to_map.transform.rotation);
    rclcpp::Time stamp         = this->now ();
    double       length        = std::hypot (gx_base, gy_base);
    int          n             = std::max (2, static_cast<int> (length / path_step_m_) + 1);

    for (int i = 0; i < n; ++i) {
        double t      = static_cast<double> (i) / static_cast<double> (n - 1);
        double x_base = gx_base * t;
        double y_base = gy_base * t;
        double yaw_base = gyaw_base * t;

        auto [x_map, y_map] = transform_point (x_base, y_base, tf_base_to_map);
        double yaw_map = yaw_base + robot_yaw_map;

        geometry_msgs::msg::PoseStamped ps;
        ps.header.frame_id    = "map";
        ps.header.stamp       = stamp;
        ps.pose.position.x    = x_map;
        ps.pose.position.y    = y_map;
        ps.pose.orientation.z = std::sin (yaw_map * 0.5);
        ps.pose.orientation.w = std::cos (yaw_map * 0.5);
        path.poses.push_back (ps);
    }

    return path;
}

double bottle_collector::quat_to_yaw (const geometry_msgs::msg::Quaternion &q) {
    return std::atan2 (2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z));
}

double bottle_collector::normalize_angle (double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

std::pair<double, double> bottle_collector::transform_point (double x, double y, const geometry_msgs::msg::TransformStamped &tf) {
    double tx  = tf.transform.translation.x;
    double ty  = tf.transform.translation.y;
    double yaw = quat_to_yaw (tf.transform.rotation);
    double c   = std::cos (yaw);
    double s   = std::sin (yaw);
    return {c * x - s * y + tx, s * x + c * y + ty};
}

std::pair<double, double> bottle_collector::inverse_transform_point (double x, double y, const geometry_msgs::msg::TransformStamped &tf) {
    double tx  = tf.transform.translation.x;
    double ty  = tf.transform.translation.y;
    double yaw = quat_to_yaw (tf.transform.rotation);
    double c   = std::cos (yaw);
    double s   = std::sin (yaw);
    double dx  = x - tx;
    double dy  = y - ty;
    return {c * dx + s * dy, -s * dx + c * dy};
}

}  // namespace bottle_collector

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (bottle_collector::bottle_collector)
