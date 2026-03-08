#include "krb2026b_duck_detection/duck_detection.hpp"

namespace duck_detection {
duck_detection::duck_detection (const rclcpp::NodeOptions &options) : Node ("duck_detection", options), cam_info_ready_ (false) {
    this->declare_parameter<std::string> ("model_path", "last_int8_openvino_model/last.xml");

    std::string model_path = this->get_parameter ("model_path").as_string ();
    ov::Core    core;
    auto        model = core.read_model (model_path);
    compiled_model_   = core.compile_model (model, "CPU");
    output_layer_     = compiled_model_.output (0);

    image_pub_ = create_publisher<sensor_msgs::msg::Image> ("duck_bbox_image", 10);
    point_pub_ = create_publisher<geometry_msgs::msg::PointStamped> ("duck_position", 10);

    color_sub_   = create_subscription<sensor_msgs::msg::Image> ("color_image_raw", 10, std::bind (&duck_detection::colorCallback, this, std::placeholders::_1));
    depth_sub_   = create_subscription<sensor_msgs::msg::Image> ("aligned_depth_to_color_image_raw", 10, std::bind (&duck_detection::depthCallback, this, std::placeholders::_1));
    caminfo_sub_ = create_subscription<sensor_msgs::msg::CameraInfo> ("camera_info", 10, std::bind (&duck_detection::cameraInfoCallback, this, std::placeholders::_1));
    tf_buffer_   = std::make_unique<tf2_ros::Buffer> (this->get_clock ());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener> (*tf_buffer_);

    RCLCPP_INFO (get_logger (), "duck detection Node Started");
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

    double use_x = 0.0;
    double use_y = 0.0;
    double use_z = 0.0;

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
        double z = depth / 1000.0;  // mm → m
        double x = (u - cx0_) * z / fx_;
        double y = (v - cy0_) * z / fy_;
        if (use_x == 0 || x > use_x) {
            use_x = x;
            use_y = y;
            use_z = z;
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
    if (use_z > 5.0) return;

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
    if (map_point.point.x < 3.0 || map_point.point.x > 4.0) return;
    point_pub_->publish (map_point);
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