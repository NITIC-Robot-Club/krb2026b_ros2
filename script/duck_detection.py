#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

import cv2
import numpy as np
from openvino import Core


MODEL_PATH = "last_int8_openvino_model/last.xml"
IMG_SIZE = 320
CONF_THRES = 0.4


class OpenVINO3DNode(Node):

    def __init__(self):
        super().__init__("openvino_3d_node")

        self.bridge = CvBridge()
        self.depth_image = None

        self.fx = None
        self.fy = None
        self.cx0 = None
        self.cy0 = None

        # ===== OpenVINO =====
        core = Core()
        model = core.read_model(MODEL_PATH)
        self.compiled_model = core.compile_model(model, "CPU")
        self.output_layer = self.compiled_model.output(0)

        # ===== Publisher =====
        self.image_pub = self.create_publisher(
            Image,
            "/detection/duck_bbox_image",
            10)

        # ===== Subscriptions =====
        self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",
            self.color_callback,
            10)

        self.create_subscription(
            Image,
            "/camera/camera/aligned_depth_to_color/image_raw",
            self.depth_callback,
            10)

        self.create_subscription(
            CameraInfo,
            "/camera/camera/color/camera_info",
            self.camera_info_callback,
            10)

        self.get_logger().info("OpenVINO 3D Node (Publish Mode) Started")

    # =====================================
    def camera_info_callback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx0 = msg.k[2]
        self.cy0 = msg.k[5]

    # =====================================
    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding="passthrough")

    # =====================================
    def color_callback(self, msg):

        if self.depth_image is None or self.fx is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        H, W = frame.shape[:2]

        # ===== 前処理 =====
        img = cv2.resize(frame, (IMG_SIZE, IMG_SIZE))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = img.astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))
        img = np.expand_dims(img, axis=0)

        # ===== 推論 =====
        results = self.compiled_model([img])[self.output_layer]
        detections = results[0]

        scale_x = W / IMG_SIZE
        scale_y = H / IMG_SIZE

        for det in detections:
            x1, y1, x2, y2, score, cls = det

            if score < CONF_THRES:
                continue

            x1 = int(x1 * scale_x)
            y1 = int(y1 * scale_y)
            x2 = int(x2 * scale_x)
            y2 = int(y2 * scale_y)

            u = int((x1 + x2) / 2)
            v = int(y1 + (y2 - y1) * 0.6)

            # ===== depth 安定取得 =====
            h, w = self.depth_image.shape
            r = 2

            u_min = max(u - r, 0)
            u_max = min(u + r + 1, w)
            v_min = max(v - r, 0)
            v_max = min(v + r + 1, h)

            depth_roi = self.depth_image[v_min:v_max, u_min:u_max]
            valid_depth = depth_roi[depth_roi > 0]

            if len(valid_depth) < 1:
                continue

            depth = np.median(valid_depth)
            z = float(depth) / 1000.0

            if z <= 0:
                continue

            # ===== 3D計算 =====
            X = (u - self.cx0) * z / self.fx
            Y = (v - self.cy0) * z / self.fy
            Z = z

            # ===== 描画 =====
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

            text = f"X:{X:.2f} Y:{Y:.2f} Z:{Z:.2f}"
            cv2.putText(frame,
                        text,
                        (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2)

        # ===== Publish =====
        out_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        out_msg.header = msg.header  # タイムスタンプ引き継ぎ
        self.image_pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OpenVINO3DNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()