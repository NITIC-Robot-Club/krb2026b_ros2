#include "krb2026b_duck_detection/duck_detection.hpp"

namespace duck_detection {
duck_detection::duck_detection (const rclcpp::NodeOptions &options) : Node ("duck_detection", options), cam_info_ready_ (false) {
    this->declare_parameter<std::string> ("model_path", "last_int8_openvino_model/last.xml");

    std::string model_path = this->get_parameter ("model_path").as_string ();
    ov::Core    core;
    auto        model = core.read_model (model_path);
    compiled_model_   = core.compile_model (model, "CPU");
    output_layer_     = compiled_model_.output (0);

    image_pub_ = create_publisher<sensor_msgs::msg::Image> ("/detection/duck_bbox_image", 10);
    point_pub_ = create_publisher<geometry_msgs::msg::PointStamped> ("/detection/duck_position", 10);
    path_pub_  = create_publisher<natto_msgs::msg::SpeedPath> ("/planning/speed_path", 10);

    color_sub_   = create_subscription<sensor_msgs::msg::Image> ("/camera/camera/color/image_raw", 10, std::bind (&duck_detection::colorCallback, this, std::placeholders::_1));
    depth_sub_   = create_subscription<sensor_msgs::msg::Image> ("/camera/camera/aligned_depth_to_color/image_raw", 10, std::bind (&duck_detection::depthCallback, this, std::placeholders::_1));
    caminfo_sub_ = create_subscription<sensor_msgs::msg::CameraInfo> ("/camera/camera/color/camera_info", 10, std::bind (&duck_detection::cameraInfoCallback, this, std::placeholders::_1));
    current_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("/localization/current_pose", 10, std::bind(&duck_detection::currentPoseCallback, this,std::placeholders::_1));

    tf_buffer_   = std::make_unique<tf2_ros::Buffer> (this->get_clock ());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener> (*tf_buffer_);

    timer_ = this->create_wall_timer (std::chrono::milliseconds (100), std::bind (&duck_detection::timerCallback, this));

    RCLCPP_INFO (get_logger (), "duck detection Node Started");
}
void duck_detection::timerCallback () {
    if (map_point.point.x == 0.0 && map_point.point.y == 0.0 && map_point.point.z == 0.0) return;
    geometry_msgs::msg::PoseStamped goal_pose;
    goal_pose.header.stamp    = this->now ();
    goal_pose.header.frame_id = "map";

    goal_pose.pose.position.x = map_point.point.x - 0.9;  // 少し手前に行く
    goal_pose.pose.position.y = map_point.point.y;
    goal_pose.pose.position.z = 0.0;

    goal_pose.pose.orientation.x = 0.0;
    goal_pose.pose.orientation.y = 0.0;
    goal_pose.pose.orientation.z = 0.0;
    goal_pose.pose.orientation.w = 1.0;

    planningPath (goal_pose);
}
void duck_detection::colorCallback (const sensor_msgs::msg::Image::SharedPtr msg) {
    if (depth_image_.empty () || !cam_info_ready_) return;

    auto    cv_ptr = cv_bridge::toCvCopy (msg, "bgr8");
    cv::Mat frame  = cv_ptr->image;

    int H = frame.rows;
    int W = frame.cols;

    // ===== Preprocess =====
    cv::Mat resized;
    cv::resize (frame, resized, cv::Size (IMG_SIZE, IMG_SIZE));
    cv::cvtColor (resized, resized, cv::COLOR_BGR2RGB);
    resized.convertTo (resized, CV_32F, 1.0 / 255.0);

    // HWC → CHW
    std::vector<float> input_data (3 * IMG_SIZE * IMG_SIZE);

    for (int c = 0; c < 3; ++c) {
        for (int y = 0; y < IMG_SIZE; ++y) {
            for (int x = 0; x < IMG_SIZE; ++x) {
                input_data[c * IMG_SIZE * IMG_SIZE + y * IMG_SIZE + x] = resized.at<cv::Vec3f> (y, x)[c];
            }
        }
    }

    ov::Tensor input_tensor (ov::element::f32, {1, 3, IMG_SIZE, IMG_SIZE}, input_data.data ());

    auto infer_request = compiled_model_.create_infer_request ();
    infer_request.set_input_tensor (input_tensor);
    infer_request.infer ();

    // ===== Postprocess =====
    auto output = infer_request.get_output_tensor ();
    auto shape  = output.get_shape ();  // [1, 2100, 6]

    float *data = output.data<float> ();

    int num_boxes = shape[1];  // 2100
    int elements  = shape[2];  // 6

    float scale_x = static_cast<float> (W) / IMG_SIZE;
    float scale_y = static_cast<float> (H) / IMG_SIZE;

    double use_x      = 0.0;
    double use_y      = 0.0;
    double use_z      = 0.0;
    double best_score = 0.0;

    for (int i = 0; i < num_boxes; ++i) {
        float x1    = data[i * elements + 0];
        float y1    = data[i * elements + 1];
        float x2    = data[i * elements + 2];
        float y2    = data[i * elements + 3];
        float score = data[i * elements + 4];
        float cls   = data[i * elements + 5];

        if (score < CONF_THRES) continue;

        x1 *= scale_x;
        y1 *= scale_y;
        x2 *= scale_x;
        y2 *= scale_y;

        // ===== 中心点計算 =====
        int u = static_cast<int> ((x1 + x2) / 2.0);
        int v = static_cast<int> (y1 + (y2 - y1) * 0.6);

        // ===== depth 安定取得 =====
        int h = depth_image_.rows;
        int w = depth_image_.cols;
        int r = 2;

        int u_min = std::max (u - r, 0);
        int u_max = std::min (u + r + 1, w);
        int v_min = std::max (v - r, 0);
        int v_max = std::min (v + r + 1, h);

        if (u_min >= u_max || v_min >= v_max) continue;

        // depthは通常16UC1（mm）
        cv::Mat depth_roi = depth_image_ (cv::Range (v_min, v_max), cv::Range (u_min, u_max));

        std::vector<double> valid_depths;

        for (int yy = 0; yy < depth_roi.rows; ++yy) {
            for (int xx = 0; xx < depth_roi.cols; ++xx) {
                uint16_t d = depth_roi.at<uint16_t> (yy, xx);

                if (d > 0) valid_depths.push_back (static_cast<double> (d));
            }
        }

        if (valid_depths.empty ()) continue;

        // median計算
        std::nth_element (valid_depths.begin (), valid_depths.begin () + valid_depths.size () / 2, valid_depths.end ());

        double depth = valid_depths[valid_depths.size () / 2];

        if (depth <= 0.0) continue;

        // ===== 3D計算 =====
        double z        = depth / 1000.0;  // mm → m
        double x        = (u - cx0_) * z / fx_;
        double y        = (v - cy0_) * z / fy_;
        double distance = std::sqrt (x * x + y * y + z * z);
        if (distance > best_score) {
            best_score = distance;
            use_x      = x;
            use_y      = y;
            use_z      = z;
        }
        // ===== 描画 =====
        cv::rectangle (frame, cv::Point (x1, y1), cv::Point (x2, y2), cv::Scalar (0, 255, 0), 2);

        std::ostringstream oss;
        oss << std::fixed << std::setprecision (2) << "X:" << x << " Y:" << y << " Z:" << z;

        cv::putText (frame, oss.str (), cv::Point (x1, y1 - 10), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar (0, 255, 0), 2);
    }

    auto out_msg = cv_bridge::CvImage (msg->header, "bgr8", frame).toImageMsg ();
    image_pub_->publish (*out_msg);

    if (use_x == 0.0 && use_y == 0.0 && use_z == 0.0) return;

    geometry_msgs::msg::PointStamped point_msg;
    point_msg.header  = msg->header;
    point_msg.point.x = use_x;
    point_msg.point.y = use_y;
    point_msg.point.z = use_z;
    try {
        map_point = tf_buffer_->transform (point_msg, "map", tf2::durationFromSec (0.1));

    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN (this->get_logger (), "%s", ex.what ());
        return;
    }
    point_pub_->publish (map_point);
}
void duck_detection::planningPath(geometry_msgs::msg::PoseStamped goal_pose)
{
    if (current_pose_.pose.position.x == 0.0 &&
        current_pose_.pose.position.y == 0.0 &&
        current_pose_.pose.position.z == 0.0) return;

    RCLCPP_INFO(get_logger(), "start planning");

    double v_max = 0.8;
    double w_max = 1.5;

    natto_msgs::msg::SpeedPath speed_path;
    speed_path.header.stamp    = this->now();
    speed_path.header.frame_id = "map";

    double current_x   = current_pose_.pose.position.x;
    double current_y   = current_pose_.pose.position.y;
    double current_yaw = quat_to_yaw(current_pose_.pose.orientation);

    double goal_x   = goal_pose.pose.position.x;
    double goal_y   = goal_pose.pose.position.y;
    double goal_yaw = quat_to_yaw(goal_pose.pose.orientation);

    double dx   = goal_x - current_x;
    double dy   = goal_y - current_y;
    double dyaw = goal_yaw - current_yaw;
    double goal_direction = std::atan2(dy, dx);

    while (dyaw >  M_PI) dyaw -= 2.0 * M_PI;
    while (dyaw < -M_PI) dyaw += 2.0 * M_PI;

    double total_dist = std::hypot(dx, dy);
    double step = 0.05;

    int N = std::max(2, static_cast<int>(total_dist / step));

    for (int i = 0; i <= N; i++)
    {
        double s = static_cast<double>(i) / N;

        // ===== V字プロファイル =====
        double tri = (s < 0.5) ? 2.0*s : 2.0*(1.0-s);
        if (tri < 0.0) tri = 0.0;

        double v = v_max * tri;
        double w = w_max * tri;

        // ===== 同時補間 =====
        double x   = current_x + s * dx;
        double y   = current_y + s * dy;
        double yaw = current_yaw + s * dyaw;

        // ===== Pose =====
        geometry_msgs::msg::PoseStamped pose;
        pose.header = speed_path.header;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = std::sin(yaw * 0.5);
        pose.pose.orientation.w = std::cos(yaw * 0.5);

        speed_path.path.push_back(pose);

        // ===== Twist =====
        geometry_msgs::msg::TwistStamped twist;
        twist.header = speed_path.header;

        // ★ 超重要：その時点の yaw を使う
        twist.twist.linear.x = v * std::cos(goal_direction - yaw);
        twist.twist.linear.y = v * std::sin(goal_direction - yaw);
        twist.twist.linear.z = 0.0;

        twist.twist.angular.x = 0.0;
        twist.twist.angular.y = 0.0;
        twist.twist.angular.z = (dyaw >= 0.0 ? w : -w);

        speed_path.twist.push_back(twist);
    }

    path_pub_->publish(speed_path);
}
void duck_detection::currentPoseCallback (const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    current_pose_ = *msg;
}
void duck_detection::cameraInfoCallback (const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    fx_             = msg->k[0];
    fy_             = msg->k[4];
    cx0_            = msg->k[2];
    cy0_            = msg->k[5];
    cam_info_ready_ = true;
}

void duck_detection::depthCallback (const sensor_msgs::msg::Image::SharedPtr msg) {
    depth_image_ = cv_bridge::toCvCopy (msg, msg->encoding)->image;
}
}  // namespace duck_detection

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (duck_detection::duck_detection)