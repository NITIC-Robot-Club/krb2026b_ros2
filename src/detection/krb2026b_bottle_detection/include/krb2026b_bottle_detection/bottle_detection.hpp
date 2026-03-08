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

#ifndef __BOTTLE_DETECTION_HPP__
#define __BOTTLE_DETECTION_HPP__

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace bottle_detection {

class bottle_detection : public rclcpp::Node {
   public:
    bottle_detection (const rclcpp::NodeOptions &node_options);
    ~bottle_detection ();

   private:
    static constexpr double BOTTLE_RADIUS = 0.032;
    static constexpr double PAIR_DIST_MIN = 0.3;
    static constexpr double PAIR_DIST_MAX = 0.5;
    static constexpr double PAIR_DX_MAX   = 0.1;
    static constexpr double YAW_OFFSET    = -M_PI / 2.0;

        using Point2D    = std::pair<double, double>;
        using Cluster    = std::vector<Point2D>;
        using BottlePair = std::pair<Point2D, Point2D>;

    std::vector<double> detect_area_global_x_;
    std::vector<double> detect_area_global_y_;
    std::vector<double> exclude_footprint_x_;
    std::vector<double> exclude_footprint_y_;
    double              cluster_dist_thresh_;
    int                 cluster_min_pts_;
    bool                reverse_y_;
    double              reverse_y_offset_;
    double              lpf_alpha_;
    double              lpf_match_dist_thresh_;
    std::vector<Point2D> previous_filtered_centers_;

    void scan_callback (const sensor_msgs::msg::LaserScan::SharedPtr msg);

    double  quat_to_yaw (const geometry_msgs::msg::Quaternion &q);
    Point2D transform_point (double x, double y, const geometry_msgs::msg::TransformStamped &tf);
    bool    point_in_polygon (double x, double y, const std::vector<double> &xs, const std::vector<double> &ys);

    std::vector<Point2D> scan_to_filtered_points (const sensor_msgs::msg::LaserScan::SharedPtr &msg, const geometry_msgs::msg::TransformStamped &tf_lidar_to_base, const geometry_msgs::msg::TransformStamped &tf_base_to_map);

    std::vector<Cluster> cluster_points (const std::vector<Point2D> &points);

    Point2D centroid (const Cluster &cluster);

    std::vector<Point2D> low_pass_filter_centers (const std::vector<Point2D> &centers);

    Point2D                 compensate_center (const Point2D &center_map, const geometry_msgs::msg::TransformStamped &tf_base_to_map);
    std::vector<BottlePair> find_pairs (const std::vector<Point2D> &centers);
    Point2D                 pair_center (const BottlePair &pair);
    double                  pair_yaw (const BottlePair &pair);

    void publish_markers (const std::vector<BottlePair> &pairs, const rclcpp::Time &stamp);
    void publish_empty_markers (const rclcpp::Time &stamp);

    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr        bottle_pairs_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
};

}  // namespace bottle_detection

#endif  // __BOTTLE_DETECTION_HPP__
