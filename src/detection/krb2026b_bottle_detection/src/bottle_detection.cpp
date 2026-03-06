#include "krb2026b_bottle_detection/bottle_detection.hpp"

namespace bottle_detection {

bottle_detection::bottle_detection (const rclcpp::NodeOptions &node_options) : Node ("bottle_detection", node_options) {
    detect_area_global_x_ = this->declare_parameter<std::vector<double>> ("detect_area_global_x", {1.7, 1.7, 3.0, 3.0});
    detect_area_global_y_ = this->declare_parameter<std::vector<double>> ("detect_area_global_y", {0.1, 1.0, 1.0, 0.1});
    exclude_footprint_x_  = this->declare_parameter<std::vector<double>> ("exclude_footprint_x", {-0.25, 0.3, 0.3, -0.25});
    exclude_footprint_y_  = this->declare_parameter<std::vector<double>> ("exclude_footprint_y", {-0.3, -0.3, 0.3, 0.3});
    cluster_dist_thresh_  = this->declare_parameter<double> ("cluster_dist_thresh", 0.1);
    cluster_min_pts_      = static_cast<int> (this->declare_parameter<int> ("cluster_min_pts", 3));

    tf_buffer_   = std::make_shared<tf2_ros::Buffer> (this->get_clock ());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener> (*tf_buffer_);

    bottle_pairs_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray> ("bottle_pairs", 10);
    marker_publisher_       = this->create_publisher<visualization_msgs::msg::MarkerArray> ("bottle_markers", 10);

    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan> ("/sensing/lidar/laserscan/rear", rclcpp::SensorDataQoS (), std::bind (&bottle_detection::scan_callback, this, std::placeholders::_1));

    RCLCPP_INFO (this->get_logger (), "bottle_detection node has been initialized.");
    RCLCPP_INFO (this->get_logger (), "cluster_dist_thresh: %f", cluster_dist_thresh_);
    RCLCPP_INFO (this->get_logger (), "cluster_min_pts: %d", cluster_min_pts_);
    for (size_t i = 0; i < detect_area_global_x_.size (); ++i) {
        RCLCPP_INFO (this->get_logger (), "detect_area_global_x[%zu]: %f", i, detect_area_global_x_[i]);
    }
    for (size_t i = 0; i < detect_area_global_y_.size (); ++i) {
        RCLCPP_INFO (this->get_logger (), "detect_area_global_y[%zu]: %f", i, detect_area_global_y_[i]);
    }
    for (size_t i = 0; i < exclude_footprint_x_.size (); ++i) {
        RCLCPP_INFO (this->get_logger (), "exclude_footprint_x[%zu]: %f", i, exclude_footprint_x_[i]);
    }
    for (size_t i = 0; i < exclude_footprint_y_.size (); ++i) {
        RCLCPP_INFO (this->get_logger (), "exclude_footprint_y[%zu]: %f", i, exclude_footprint_y_[i]);
    }
}

bottle_detection::~bottle_detection () {}

double bottle_detection::quat_to_yaw (const geometry_msgs::msg::Quaternion &q) {
    return std::atan2 (2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z));
}

bottle_detection::Point2D bottle_detection::transform_point (double x, double y, const geometry_msgs::msg::TransformStamped &tf) {
    double tx  = tf.transform.translation.x;
    double ty  = tf.transform.translation.y;
    double yaw = quat_to_yaw (tf.transform.rotation);
    double c   = std::cos (yaw);
    double s   = std::sin (yaw);
    return {c * x - s * y + tx, s * x + c * y + ty};
}

static std::pair<double, double> inverse_transform_point (double x, double y, const geometry_msgs::msg::TransformStamped &tf) {
    double tx  = tf.transform.translation.x;
    double ty  = tf.transform.translation.y;
    double yaw = std::atan2 (2.0 * (tf.transform.rotation.w * tf.transform.rotation.z), 1.0 - 2.0 * (tf.transform.rotation.z * tf.transform.rotation.z));
    double c   = std::cos (yaw);
    double s   = std::sin (yaw);
    double dx  = x - tx;
    double dy  = y - ty;
    return {c * dx + s * dy, -s * dx + c * dy};
}

bool bottle_detection::point_in_polygon (double x, double y, const std::vector<double> &xs, const std::vector<double> &ys) {
    bool   inside = false;
    size_t n      = xs.size ();
    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        if (((ys[i] > y) != (ys[j] > y)) && (x < (xs[j] - xs[i]) * (y - ys[i]) / (ys[j] - ys[i] + 1e-9) + xs[i])) {
            inside = !inside;
        }
    }
    return inside;
}

std::vector<bottle_detection::Point2D> bottle_detection::scan_to_filtered_points (
    const sensor_msgs::msg::LaserScan::SharedPtr &msg, const geometry_msgs::msg::TransformStamped &tf_lidar_to_base, const geometry_msgs::msg::TransformStamped &tf_base_to_map) {
    std::vector<Point2D> result;
    double               angle = msg->angle_min;

    for (const float r : msg->ranges) {
        if (r > msg->range_min && r < msg->range_max) {
            double lx = r * std::cos (angle);
            double ly = r * std::sin (angle);

            auto [bx, by] = transform_point (lx, ly, tf_lidar_to_base);

            if (!point_in_polygon (bx, by, exclude_footprint_x_, exclude_footprint_y_)) {
                auto [mx, my] = transform_point (bx, by, tf_base_to_map);

                if (point_in_polygon (mx, my, detect_area_global_x_, detect_area_global_y_)) {
                    result.emplace_back (mx, my);
                }
            }
        }
        angle += msg->angle_increment;
    }
    return result;
}

std::vector<bottle_detection::Cluster> bottle_detection::cluster_points (const std::vector<Point2D> &points) {
    std::vector<Cluster> clusters;
    Cluster              current;

    for (const auto &p : points) {
        if (current.empty ()) {
            current.push_back (p);
            continue;
        }
        double dx = p.first - current.back ().first;
        double dy = p.second - current.back ().second;
        if (std::hypot (dx, dy) < cluster_dist_thresh_) {
            current.push_back (p);
        } else {
            if (static_cast<int> (current.size ()) >= cluster_min_pts_) {
                clusters.push_back (current);
            }
            current = {p};
        }
    }
    if (static_cast<int> (current.size ()) >= cluster_min_pts_) {
        clusters.push_back (current);
    }
    return clusters;
}

bottle_detection::Point2D bottle_detection::centroid (const Cluster &cluster) {
    double sx = 0.0, sy = 0.0;
    for (const auto &p : cluster) {
        sx += p.first;
        sy += p.second;
    }
    double n = static_cast<double> (cluster.size ());
    return {sx / n, sy / n};
}

bottle_detection::Point2D bottle_detection::compensate_center (const Point2D &center_map, const geometry_msgs::msg::TransformStamped &tf_base_to_map) {
    auto [bx, by] = inverse_transform_point (center_map.first, center_map.second, tf_base_to_map);
    double d      = std::hypot (bx, by);
    if (d < 1e-6) {
        return center_map;
    }
    double scale = (d + BOTTLE_RADIUS) / d;
    double bx_c  = bx * scale;
    double by_c  = by * scale;

    return transform_point (bx_c, by_c, tf_base_to_map);
}

std::vector<bottle_detection::BottlePair> bottle_detection::find_pairs (const std::vector<Point2D> &centers) {
    std::vector<BottlePair> pairs;
    for (size_t i = 0; i < centers.size (); ++i) {
        for (size_t j = i + 1; j < centers.size (); ++j) {
            double dx = std::abs (centers[i].first - centers[j].first);
            double dy = std::abs (centers[i].second - centers[j].second);
            double d  = std::hypot (dx, dy);
            if (d > PAIR_DIST_MIN && d < PAIR_DIST_MAX && dx < PAIR_DX_MAX) {
                pairs.emplace_back (centers[i], centers[j]);
            }
        }
    }
    return pairs;
}

bottle_detection::Point2D bottle_detection::pair_center (const BottlePair &pair) {
    return {(pair.first.first + pair.second.first) * 0.5, (pair.first.second + pair.second.second) * 0.5};
}

double bottle_detection::pair_yaw (const BottlePair &pair) {
    double yaw = std::atan2 (pair.second.second - pair.first.second, pair.second.first - pair.first.first) + YAW_OFFSET;
    if (std::cos (yaw) < 0.0) {
        yaw += M_PI;
    }
    return yaw;
}

void bottle_detection::publish_empty_markers (const rclcpp::Time &stamp) {
    visualization_msgs::msg::MarkerArray ma;
    int                                  mid = 0;

    visualization_msgs::msg::Marker del;
    del.header.frame_id = "map";
    del.header.stamp    = stamp;
    del.ns              = "bottle";
    del.action          = visualization_msgs::msg::Marker::DELETEALL;
    ma.markers.push_back (del);

    visualization_msgs::msg::Marker del2;
    del2.header.frame_id = "map";
    del2.header.stamp    = stamp;
    del2.ns              = "pair_dir";
    del2.action          = visualization_msgs::msg::Marker::DELETEALL;
    ma.markers.push_back (del2);

    {
        visualization_msgs::msg::Marker area_m;
        area_m.header.frame_id = "map";
        area_m.header.stamp    = stamp;
        area_m.ns              = "detect_area";
        area_m.id              = mid++;
        area_m.type            = visualization_msgs::msg::Marker::LINE_STRIP;
        area_m.scale.x         = 0.02;
        area_m.color.r         = 0.0f;
        area_m.color.g         = 1.0f;
        area_m.color.b         = 0.0f;
        area_m.color.a         = 1.0f;
        size_t n               = detect_area_global_x_.size ();
        for (size_t i = 0; i <= n; ++i) {
            geometry_msgs::msg::Point pt;
            pt.x = detect_area_global_x_[i % n];
            pt.y = detect_area_global_y_[i % n];
            area_m.points.push_back (pt);
        }
        ma.markers.push_back (area_m);
    }

    {
        visualization_msgs::msg::Marker fp_m;
        fp_m.header.frame_id = "base_link";
        fp_m.header.stamp    = stamp;
        fp_m.ns              = "exclude_footprint";
        fp_m.id              = mid++;
        fp_m.type            = visualization_msgs::msg::Marker::LINE_STRIP;
        fp_m.scale.x         = 0.02;
        fp_m.color.r         = 1.0f;
        fp_m.color.g         = 0.0f;
        fp_m.color.b         = 0.0f;
        fp_m.color.a         = 1.0f;
        size_t nf            = exclude_footprint_x_.size ();
        for (size_t i = 0; i <= nf; ++i) {
            geometry_msgs::msg::Point pt;
            pt.x = exclude_footprint_x_[i % nf];
            pt.y = exclude_footprint_y_[i % nf];
            fp_m.points.push_back (pt);
        }
        ma.markers.push_back (fp_m);
    }

    marker_publisher_->publish (ma);
}

void bottle_detection::publish_markers (const std::vector<BottlePair> &pairs, const rclcpp::Time &stamp) {
    visualization_msgs::msg::MarkerArray ma;
    int                                  mid = 0;

    {
        visualization_msgs::msg::Marker area_m;
        area_m.header.frame_id = "map";
        area_m.header.stamp    = stamp;
        area_m.ns              = "detect_area";
        area_m.id              = mid++;
        area_m.type            = visualization_msgs::msg::Marker::LINE_STRIP;
        area_m.scale.x         = 0.02;
        area_m.color.r         = 0.0f;
        area_m.color.g         = 1.0f;
        area_m.color.b         = 0.0f;
        area_m.color.a         = 1.0f;
        size_t n               = detect_area_global_x_.size ();
        for (size_t i = 0; i <= n; ++i) {
            geometry_msgs::msg::Point pt;
            pt.x = detect_area_global_x_[i % n];
            pt.y = detect_area_global_y_[i % n];
            area_m.points.push_back (pt);
        }
        ma.markers.push_back (area_m);
    }

    {
        visualization_msgs::msg::Marker fp_m;
        fp_m.header.frame_id = "base_link";
        fp_m.header.stamp    = stamp;
        fp_m.ns              = "exclude_footprint";
        fp_m.id              = mid++;
        fp_m.type            = visualization_msgs::msg::Marker::LINE_STRIP;
        fp_m.scale.x         = 0.02;
        fp_m.color.r         = 1.0f;
        fp_m.color.g         = 0.0f;
        fp_m.color.b         = 0.0f;
        fp_m.color.a         = 1.0f;
        size_t nf            = exclude_footprint_x_.size ();
        for (size_t i = 0; i <= nf; ++i) {
            geometry_msgs::msg::Point pt;
            pt.x = exclude_footprint_x_[i % nf];
            pt.y = exclude_footprint_y_[i % nf];
            fp_m.points.push_back (pt);
        }
        ma.markers.push_back (fp_m);
    }

    for (const auto &pair : pairs) {
        for (const auto &[cx, cy] : std::initializer_list<Point2D>{pair.first, pair.second}) {
            visualization_msgs::msg::Marker m;
            m.header.frame_id    = "map";
            m.header.stamp       = stamp;
            m.ns                 = "bottle";
            m.id                 = mid++;
            m.type               = visualization_msgs::msg::Marker::CYLINDER;
            m.pose.position.x    = cx;
            m.pose.position.y    = cy;
            m.pose.position.z    = 0.1;
            m.pose.orientation.w = 1.0;
            m.scale.x            = 0.064;
            m.scale.y            = 0.064;
            m.scale.z            = 0.20;
            m.color.r            = 0.0f;
            m.color.g            = 0.4f;
            m.color.b            = 1.0f;
            m.color.a            = 0.8f;
            m.lifetime.sec       = 0;
            m.lifetime.nanosec   = 200000000u;
            ma.markers.push_back (m);
        }

        auto [pcx, pcy]                     = pair_center (pair);
        double                          yaw = pair_yaw (pair);
        visualization_msgs::msg::Marker arrow;
        arrow.header.frame_id    = "map";
        arrow.header.stamp       = stamp;
        arrow.ns                 = "pair_dir";
        arrow.id                 = mid++;
        arrow.type               = visualization_msgs::msg::Marker::ARROW;
        arrow.pose.position.x    = pcx;
        arrow.pose.position.y    = pcy;
        arrow.pose.orientation.z = std::sin (yaw * 0.5);
        arrow.pose.orientation.w = std::cos (yaw * 0.5);
        arrow.scale.x            = 0.2;
        arrow.scale.y            = 0.03;
        arrow.scale.z            = 0.03;
        arrow.color.r            = 1.0f;
        arrow.color.g            = 1.0f;
        arrow.color.b            = 0.0f;
        arrow.color.a            = 0.9f;
        arrow.lifetime.sec       = 0;
        arrow.lifetime.nanosec   = 200000000u;
        ma.markers.push_back (arrow);
    }

    marker_publisher_->publish (ma);
}

void bottle_detection::scan_callback (const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    geometry_msgs::msg::TransformStamped tf_lidar_to_base;
    geometry_msgs::msg::TransformStamped tf_base_to_map;

    try {
        tf_lidar_to_base = tf_buffer_->lookupTransform ("base_link", msg->header.frame_id, rclcpp::Time (0));
        tf_base_to_map   = tf_buffer_->lookupTransform ("map", "base_link", rclcpp::Time (0));
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE (this->get_logger (), *this->get_clock (), 2000, "TF lookup failed: %s", ex.what ());
        publish_empty_markers (msg->header.stamp);
        return;
    }

    auto points = scan_to_filtered_points (msg, tf_lidar_to_base, tf_base_to_map);
    if (points.empty ()) {
        publish_empty_markers (msg->header.stamp);
        return;
    }

    auto clusters = cluster_points (points);
    if (clusters.empty ()) {
        publish_empty_markers (msg->header.stamp);
        return;
    }

    std::vector<Point2D> centers;
    centers.reserve (clusters.size ());
    for (const auto &c : clusters) {
        centers.push_back (compensate_center (centroid (c), tf_base_to_map));
    }

    auto pairs = find_pairs (centers);
    if (pairs.empty ()) {
        publish_empty_markers (msg->header.stamp);
        return;
    }

    double robot_x = tf_base_to_map.transform.translation.x;
    double robot_y = tf_base_to_map.transform.translation.y;
    std::sort (pairs.begin (), pairs.end (), [&] (const BottlePair &a, const BottlePair &b) {
        auto [acx, acy] = pair_center (a);
        auto [bcx, bcy] = pair_center (b);
        return std::hypot (acx - robot_x, acy - robot_y) < std::hypot (bcx - robot_x, bcy - robot_y);
    });

    geometry_msgs::msg::PoseArray pa;
    pa.header.frame_id = "map";
    pa.header.stamp    = msg->header.stamp;

    for (const auto &pair : pairs) {
        auto [cx, cy] = pair_center (pair);
        double yaw    = pair_yaw (pair);

        geometry_msgs::msg::Pose p;
        p.position.x    = cx;
        p.position.y    = cy;
        p.orientation.z = std::sin (yaw * 0.5);
        p.orientation.w = std::cos (yaw * 0.5);
        pa.poses.push_back (p);
    }

    bottle_pairs_publisher_->publish (pa);
    publish_markers (pairs, msg->header.stamp);

    RCLCPP_DEBUG (this->get_logger (), "Detected %zu bottle pair(s).", pairs.size ());
}

}  // namespace bottle_detection

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (bottle_detection::bottle_detection)
