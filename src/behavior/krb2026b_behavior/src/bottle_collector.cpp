#include "krb2026b_behavior/bottle_collector.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace bottle_collector {

bottle_collector::bottle_collector (const rclcpp::NodeOptions &node_options) : Node ("bottle_collector", node_options) {
    max_velocity_mps_  = this->declare_parameter<double> ("max_velocity_mps", 0.5);
    acceleration_mps2_ = this->declare_parameter<double> ("acceleration_mps2", 0.3);
    path_step_m_       = this->declare_parameter<double> ("path_step_m", 0.05);
    offset_normal_m_   = this->declare_parameter<double> ("offset_normal_m", 0.325);
    offset_large_m_    = this->declare_parameter<double> ("offset_large_m", 0.42);
    y_thresh_m_        = this->declare_parameter<double> ("y_thresh_m", 0.02);
    yaw_thresh_rad_    = this->declare_parameter<double> ("yaw_thresh_deg", 3.0) * M_PI / 180.0;
    double frequency   = this->declare_parameter<double> ("frequency", 10.0);

    tf_buffer_   = std::make_shared<tf2_ros::Buffer> (this->get_clock ());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener> (*tf_buffer_);

    state_result_publisher_ = this->create_publisher<natto_msgs::msg::StateResult> ("state_result", 10);
    speed_path_publisher_   = this->create_publisher<natto_msgs::msg::SpeedPath> ("speed_path", 10);
    goal_pose_publisher_    = this->create_publisher<geometry_msgs::msg::PoseStamped> ("collect_goal_pose", 10);

    state_action_subscriber_ = this->create_subscription<natto_msgs::msg::StateAction> ("state_action", 10, std::bind (&bottle_collector::state_action_callback, this, std::placeholders::_1));
    bottle_pairs_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseArray> ("bottle_pairs", 10, std::bind (&bottle_collector::bottle_pairs_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer (std::chrono::duration<double> (1.0 / frequency), std::bind (&bottle_collector::timer_callback, this));

    RCLCPP_INFO (this->get_logger (), "bottle_collector node has been initialized.");
    RCLCPP_INFO (this->get_logger (), "max_velocity_mps: %f", max_velocity_mps_);
    RCLCPP_INFO (this->get_logger (), "acceleration_mps2: %f", acceleration_mps2_);
    RCLCPP_INFO (this->get_logger (), "path_step_m: %f", path_step_m_);
    RCLCPP_INFO (this->get_logger (), "offset_normal_m: %f", offset_normal_m_);
    RCLCPP_INFO (this->get_logger (), "offset_large_m: %f", offset_large_m_);
    RCLCPP_INFO (this->get_logger (), "y_thresh_m: %f", y_thresh_m_);
    RCLCPP_INFO (this->get_logger (), "yaw_thresh_deg: %f", yaw_thresh_rad_ * 180.0 / M_PI);
    RCLCPP_INFO (this->get_logger (), "frequency: %f Hz", frequency);
}

void bottle_collector::state_action_callback (const natto_msgs::msg::StateAction::SharedPtr msg) {
    if (msg->action_name == "collect_bottle") {
        pending_action_msg_ = msg;
        collecting_         = true;
        RCLCPP_INFO (this->get_logger (), "collect_bottle: action received (state_id=%ld), waiting for timer.", msg->state_id);
    }
}

void bottle_collector::timer_callback () {
    if (!collecting_) return;
    collecting_ = false;
    collect_bottle (pending_action_msg_);
}

void bottle_collector::bottle_pairs_callback (const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    latest_bottle_pairs_ = msg;
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

    auto [tx_base, ty_base] = inverse_transform_point (tx_map, ty_map, tf_base_to_map);
    double robot_yaw_map    = quat_to_yaw (tf_base_to_map.transform.rotation);
    double tyaw_base        = tyaw_map - robot_yaw_map;
    while (tyaw_base > M_PI) tyaw_base -= 2.0 * M_PI;
    while (tyaw_base < -M_PI) tyaw_base += 2.0 * M_PI;

    double approach_yaw = tyaw_base + M_PI;
    while (approach_yaw > M_PI) approach_yaw -= 2.0 * M_PI;
    while (approach_yaw < -M_PI) approach_yaw += 2.0 * M_PI;

    double gx_large  = tx_base + offset_large_m_ * std::cos (approach_yaw);
    double gy_large  = ty_base + offset_large_m_ * std::sin (approach_yaw);
    double gx_normal = tx_base + offset_normal_m_ * std::cos (approach_yaw);
    double gy_normal = ty_base + offset_normal_m_ * std::sin (approach_yaw);

    if (std::hypot (gx_large, gy_large) < 1e-3) {
        RCLCPP_WARN (this->get_logger (), "collect_bottle: goal too close to current position.");
        result.success = false;
        state_result_publisher_->publish (result);
        return;
    }

    RCLCPP_INFO (this->get_logger (), "collect_bottle: target in base_link: tx=%.3f  ty=%.3f  tyaw=%.1f deg  approach_yaw=%.1f deg", tx_base, ty_base, tyaw_base * 180.0 / M_PI, approach_yaw * 180.0 / M_PI);

    bool need_phase1 = (std::fabs (ty_base) > y_thresh_m_) || (std::fabs (tyaw_base) > yaw_thresh_rad_);
    RCLCPP_INFO (this->get_logger (), "collect_bottle: need_phase1=%d  (|ty|=%.3f vs %.3f, |tyaw|=%.1f deg vs %.1f deg)", need_phase1, std::fabs (ty_base), y_thresh_m_, std::fabs (tyaw_base) * 180.0 / M_PI, yaw_thresh_rad_ * 180.0 / M_PI);

    auto speed_path            = generate_speed_path (gx_large, gy_large, gx_normal, gy_normal, approach_yaw, need_phase1, tf_base_to_map);
    speed_path.header.stamp    = this->now ();
    speed_path.header.frame_id = "map";
    speed_path_publisher_->publish (speed_path);

    auto [goal_x_map, goal_y_map] = transform_point (gx_normal, gy_normal, tf_base_to_map);
    double gyaw_map               = approach_yaw + robot_yaw_map;

    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id    = "map";
    goal.header.stamp       = this->now ();
    goal.pose.position.x    = goal_x_map;
    goal.pose.position.y    = goal_y_map;
    goal.pose.orientation.z = std::sin (gyaw_map * 0.5);
    goal.pose.orientation.w = std::cos (gyaw_map * 0.5);
    goal_pose_publisher_->publish (goal);

    result.success = true;
    state_result_publisher_->publish (result);

    RCLCPP_INFO (this->get_logger (), "collect_bottle: SpeedPath published (%zu waypoints, need_phase1=%d).", speed_path.path.size (), need_phase1);
}

natto_msgs::msg::SpeedPath bottle_collector::generate_speed_path (
    double gx_large_base, double gy_large_base, double gx_normal_base, double gy_normal_base, double gyaw_base, bool need_phase1, const geometry_msgs::msg::TransformStamped &tf_base_to_map) {
    natto_msgs::msg::SpeedPath sp;

    double       robot_yaw_map = quat_to_yaw (tf_base_to_map.transform.rotation);
    rclcpp::Time stamp         = this->now ();

    double phase1_end_x = 0.0;
    double phase1_end_y = 0.0;

    if (need_phase1) {
        double dist1     = std::hypot (gx_large_base, gy_large_base);
        int    n1        = std::max (2, static_cast<int> (dist1 / path_step_m_) + 1);
        double curvature = (dist1 > 1e-6) ? (gyaw_base / dist1) : 0.0;

        for (int i = 0; i <= n1; ++i) {
            double t        = static_cast<double> (i) / static_cast<double> (n1);
            double x_base   = gx_large_base * t;
            double y_base   = gy_large_base * t;
            double yaw_base = gyaw_base * t;

            auto [x_map, y_map] = transform_point (x_base, y_base, tf_base_to_map);
            double yaw_map      = yaw_base + robot_yaw_map;

            geometry_msgs::msg::PoseStamped ps;
            ps.header.frame_id    = "map";
            ps.header.stamp       = stamp;
            ps.pose.position.x    = x_map;
            ps.pose.position.y    = y_map;
            ps.pose.orientation.z = std::sin (yaw_map * 0.5);
            ps.pose.orientation.w = std::cos (yaw_map * 0.5);
            sp.path.push_back (ps);

            double s = t * dist1;
            double v = trapezoid_velocity (s, dist1);

            geometry_msgs::msg::TwistStamped ts;
            ts.header.frame_id = "base_link";
            ts.header.stamp    = stamp;
            ts.twist.linear.x  = -v;
            ts.twist.angular.z = curvature * v;
            sp.twist.push_back (ts);
        }

        phase1_end_x = gx_large_base;
        phase1_end_y = gy_large_base;
    }

    double dx2   = gx_normal_base - phase1_end_x;
    double dy2   = gy_normal_base - phase1_end_y;
    double dist2 = std::hypot (dx2, dy2);
    int    n2    = std::max (2, static_cast<int> (dist2 / path_step_m_) + 1);

    double dir_x = (dist2 > 1e-6) ? (dx2 / dist2) : 0.0;
    double dir_y = (dist2 > 1e-6) ? (dy2 / dist2) : 0.0;

    double yaw_map_fixed = gyaw_base + robot_yaw_map;

    int i_start = need_phase1 ? 1 : 0;
    for (int i = i_start; i <= n2; ++i) {
        double t      = static_cast<double> (i) / static_cast<double> (n2);
        double x_base = phase1_end_x + dx2 * t;
        double y_base = phase1_end_y + dy2 * t;

        auto [x_map, y_map] = transform_point (x_base, y_base, tf_base_to_map);

        geometry_msgs::msg::PoseStamped ps;
        ps.header.frame_id    = "map";
        ps.header.stamp       = stamp;
        ps.pose.position.x    = x_map;
        ps.pose.position.y    = y_map;
        ps.pose.orientation.z = std::sin (yaw_map_fixed * 0.5);
        ps.pose.orientation.w = std::cos (yaw_map_fixed * 0.5);
        sp.path.push_back (ps);

        double s = t * dist2;
        double v = trapezoid_velocity (s, dist2);

        geometry_msgs::msg::TwistStamped ts;
        ts.header.frame_id = "base_link";
        ts.header.stamp    = stamp;
        ts.twist.linear.x  = dir_x * v;
        ts.twist.linear.y  = dir_y * v;
        ts.twist.angular.z = 0.0;
        sp.twist.push_back (ts);
    }

    return sp;
}

double bottle_collector::quat_to_yaw (const geometry_msgs::msg::Quaternion &q) {
    return std::atan2 (2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z));
}

double bottle_collector::trapezoid_velocity (double s, double total_dist) {
    if (total_dist < 1e-6) {
        return 0.0;
    }
    double v_up   = std::sqrt (2.0 * acceleration_mps2_ * s);
    double v_down = std::sqrt (2.0 * acceleration_mps2_ * std::max (total_dist - s, 0.0));
    return std::min ({max_velocity_mps_, v_up, v_down});
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
