#pragma once

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <natto_msgs/msg/state_action.hpp>
#include <natto_msgs/msg/state_result.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>

namespace duck_collector {
class duck_collector : public rclcpp::Node {
   public:
    duck_collector (const rclcpp::NodeOptions &options);

   private:
    void currentPoseCallback (const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void planningPath (geometry_msgs::msg::PoseStamped goal_pose, bool force_replan);
    void timerCallback ();
    void mapPointCallback (const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void stateActionCallback (const natto_msgs::msg::StateAction::SharedPtr msg);
    void goalReachedCallback (const std_msgs::msg::Bool::SharedPtr msg);

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr  current_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr map_point_sub_;
    rclcpp::Subscription<natto_msgs::msg::StateAction>::SharedPtr     state_action_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr              goal_reached_sub_;

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr          path_pub_;
    rclcpp::Publisher<natto_msgs::msg::StateResult>::SharedPtr state_result_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::PoseStamped         current_pose_;
    geometry_msgs::msg::PointStamped        map_point;
    natto_msgs::msg::StateAction::SharedPtr pending_action_msg_;
    bool                                    collecting_{false};

    double x_offset_;
    double y_offset_;
    double filter_gain_;
    double goal_dist_threshold_ = 0.1;  // 目標が前回の目標と近すぎる場合の距離閾値

    double prev_goal_x_;
    double prev_goal_y_;
    double path_goal_x_;
    double path_goal_y_;

    double quat_to_yaw (const geometry_msgs::msg::Quaternion &q) {
        return std::atan2 (2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z));
    }
};
}  // namespace duck_collector