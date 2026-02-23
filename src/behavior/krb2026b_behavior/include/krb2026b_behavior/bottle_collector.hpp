
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

#include "natto_msgs/msg/speed_path.hpp"
#include "natto_msgs/msg/state_action.hpp"
#include "natto_msgs/msg/state_result.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace bottle_collector {
class bottle_collector : public rclcpp::Node {
   public:
    bottle_collector (const rclcpp::NodeOptions &node_options);

   private:
    std::vector<double> detect_area_x_;
    std::vector<double> detect_area_y_;

    rclcpp::Publisher<natto_msgs::msg::StateResult>::SharedPtr         state_result_publisher_;
    rclcpp::Publisher<natto_msgs::msg::SpeedPath>::SharedPtr           speed_path_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    rclcpp::Subscription<natto_msgs::msg::StateAction>::SharedPtr      state_action_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr     pointcloud_subscriber_;
};
}  // namespace bottle_collector

#endif  // __BOTTLE_COLLECTOR_HPP__