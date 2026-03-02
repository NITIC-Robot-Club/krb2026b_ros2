#pragma once

#include "tf2/convert.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <rclcpp/rclcpp.hpp>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <natto_msgs/msg/speed_path.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
namespace duck_detection {
class duck_detection : public rclcpp::Node {
   public:
    duck_detection (const rclcpp::NodeOptions &options);

   private:
    void colorCallback (const sensor_msgs::msg::Image::SharedPtr msg);
    void depthCallback (const sensor_msgs::msg::Image::SharedPtr msg);
    void cameraInfoCallback (const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void currentPoseCallback (const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void planningPath (geometry_msgs::msg::PoseStamped goal_pose);
    void timerCallback ();

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr         color_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr         depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr    caminfo_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_sub_;

    // Publisher
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr          image_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_pub_;
    rclcpp::Publisher<natto_msgs::msg::SpeedPath>::SharedPtr       path_pub_;

    std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::TimerBase::SharedPtr timer_;

    // OpenVINO
    ov::CompiledModel          compiled_model_;
    ov::Output<const ov::Node> output_layer_;

    // Camera parameters
    double fx_, fy_, cx0_, cy0_;
    bool   cam_info_ready_;

    // Depth
    cv::Mat depth_image_;

    // Constants
    const int   IMG_SIZE   = 320;
    const float CONF_THRES = 0.4f;

    geometry_msgs::msg::PoseStamped  current_pose_;
    geometry_msgs::msg::PointStamped map_point;

    double quat_to_yaw (const geometry_msgs::msg::Quaternion &q) {
        return std::atan2 (2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z));
    }
};
}  // namespace duck_detection