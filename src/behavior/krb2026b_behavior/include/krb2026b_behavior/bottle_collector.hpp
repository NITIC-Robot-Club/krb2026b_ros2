// Copyright 2026 Kazusa Hashimoto
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef __BOTTLE_COLLECTOR_HPP__
#define __BOTTLE_COLLECTOR_HPP__

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "natto_msgs/msg/speed_path.hpp"
#include "natto_msgs/msg/state_action.hpp"
#include "natto_msgs/msg/state_result.hpp"

#include <utility>

namespace bottle_collector {

class bottle_collector : public rclcpp::Node {
   public:
    bottle_collector (const rclcpp::NodeOptions &node_options);
    ~bottle_collector ();

   private:
    double max_velocity_mps_;
    double acceleration_mps2_;
    double path_step_m_;
    double offset_normal_m_;
    double offset_large_m_;
    double y_thresh_m_;
    double yaw_thresh_rad_;

    void state_action_callback (const natto_msgs::msg::StateAction::SharedPtr msg);
    void bottle_pairs_callback (const geometry_msgs::msg::PoseArray::SharedPtr msg);
    void timer_callback ();

    void collect_bottle (const natto_msgs::msg::StateAction::SharedPtr msg);

    natto_msgs::msg::SpeedPath generate_speed_path (double gx_large_base, double gy_large_base, double gx_normal_base, double gy_normal_base, double gyaw_base, bool need_phase1, const geometry_msgs::msg::TransformStamped &tf_base_to_map);

    double                    quat_to_yaw (const geometry_msgs::msg::Quaternion &q);
    double                    trapezoid_velocity (double s, double total_dist);
    std::pair<double, double> transform_point (double x, double y, const geometry_msgs::msg::TransformStamped &tf);
    std::pair<double, double> inverse_transform_point (double x, double y, const geometry_msgs::msg::TransformStamped &tf);

    geometry_msgs::msg::PoseArray::SharedPtr latest_bottle_pairs_;
    natto_msgs::msg::StateAction::SharedPtr  pending_action_msg_;
    bool                                     collecting_{false};

    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Publisher<natto_msgs::msg::StateResult>::SharedPtr    state_result_publisher_;
    rclcpp::Publisher<natto_msgs::msg::SpeedPath>::SharedPtr      speed_path_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_publisher_;

    rclcpp::Subscription<natto_msgs::msg::StateAction>::SharedPtr  state_action_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr bottle_pairs_subscriber_;
};

}  // namespace bottle_collector

#endif  // __BOTTLE_COLLECTOR_HPP__
