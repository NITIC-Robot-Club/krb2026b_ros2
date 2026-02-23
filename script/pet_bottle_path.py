#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import (
    PoseArray, Pose, Point,
    PoseStamped
)
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros


class BottleDetector(Node):

    BOTTLE_RADIUS = 0.032
    YAW_OFFSET = -math.pi / 2.0

    def __init__(self):
        super().__init__('bottle_detector')

        # ROI (base_link)
        self.declare_parameter(
            'roi_x', [-0.8, -0.25, -0.25, -0.8]
        )
        self.declare_parameter(
            'roi_y', [-0.3, -0.3, 0.3, 0.3]
        )

        # Path parameters
        self.declare_parameter('y_thresh', 0.02)
        self.declare_parameter('yaw_thresh_deg', 3.0)
        self.declare_parameter('path_step', 0.05)

        self.sub = self.create_subscription(
            LaserScan,
            '/sensing/lidar/laserscan/rear',
            self.scan_cb,
            qos_profile_sensor_data
        )

        self.pose_pub = self.create_publisher(
            PoseArray,
            '/bottle_targets',
            10
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/bottle_markers',
            10
        )

        self.path_pub = self.create_publisher(
            Path,
            '/planning/path',
            10
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    # ----------------------------
    # 幾何
    # ----------------------------

    def quat_to_yaw(self, q):
        return math.atan2(
            2.0 * (q.w * q.z),
            1.0 - 2.0 * (q.z * q.z)
        )

    def transform_point(self, x, y, tf):
        tx = tf.transform.translation.x
        ty = tf.transform.translation.y

        q = tf.transform.rotation
        yaw = self.quat_to_yaw(q)

        c = math.cos(yaw)
        s = math.sin(yaw)

        bx = c * x - s * y + tx
        by = s * x + c * y + ty
        return bx, by

    def point_in_polygon(self, x, y, poly):
        inside = False
        n = len(poly)

        for i in range(n):
            x1, y1 = poly[i]
            x2, y2 = poly[(i + 1) % n]

            if ((y1 > y) != (y2 > y)):
                x_int = (x2 - x1) * (y - y1) / (y2 - y1 + 1e-9) + x1
                if x < x_int:
                    inside = not inside

        return inside

    # ----------------------------
    # 点群処理
    # ----------------------------

    def scan_to_points_base(self, msg, tf, roi):
        points = []
        angle = msg.angle_min

        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                lx = r * math.cos(angle)
                ly = r * math.sin(angle)

                bx, by = self.transform_point(lx, ly, tf)

                if self.point_in_polygon(bx, by, roi):
                    points.append((bx, by))

            angle += msg.angle_increment

        return points

    def cluster_points(self, points, dist_thresh=0.1, min_pts=3):
        clusters = []
        current = []

        for p in points:
            if not current:
                current = [p]
                continue

            dx = p[0] - current[-1][0]
            dy = p[1] - current[-1][1]
            if math.hypot(dx, dy) < dist_thresh:
                current.append(p)
            else:
                if len(current) >= min_pts:
                    clusters.append(current)
                current = [p]

        if len(current) >= min_pts:
            clusters.append(current)

        return clusters

    def centroid(self, cluster):
        sx = sum(p[0] for p in cluster)
        sy = sum(p[1] for p in cluster)
        n = len(cluster)
        return sx / n, sy / n

    def compensate_center(self, cx, cy):
        d = math.hypot(cx, cy)
        if d < 1e-6:
            return cx, cy
        scale = (d + self.BOTTLE_RADIUS) / d
        return cx * scale, cy * scale

    # ----------------------------
    # ペア処理
    # ----------------------------

    def find_pairs(self, centers):
        pairs = []
        for i in range(len(centers)):
            for j in range(i + 1, len(centers)):
                x1, y1 = centers[i]
                x2, y2 = centers[j]

                dx = abs(x1 - x2)
                dy = abs(y1 - y2)
                d = math.hypot(dx, dy)

                angle = math.atan2(y2 - y1, x2 - x1)

                if 0.3 < d < 0.5 and dx < 0.1:
                    pairs.append((centers[i], centers[j]))

        return pairs

    def pair_center(self, pair):
        (x1, y1), (x2, y2) = pair
        return (x1 + x2) * 0.5, (y1 + y2) * 0.5

    def pair_yaw(self, pair):
        (x1, y1), (x2, y2) = pair
        yaw = math.atan2(y2 - y1, x2 - x1) + self.YAW_OFFSET

        if math.cos(yaw) < 0.0:
            yaw += math.pi

        return yaw

    # ----------------------------
    # Path 生成
    # ----------------------------

    def generate_path_base(self, gx, gy, gyaw, step):
        length = math.hypot(gx, gy)
        n = max(2, int(length / step) + 1)

        path = []
        for i in range(n):
            t = i / (n - 1)

            x = gx * t
            y = gy * t
            yaw = gyaw * t

            ps = PoseStamped()
            ps.header.frame_id = 'base_link'
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.z = math.sin(yaw * 0.5)
            ps.pose.orientation.w = math.cos(yaw * 0.5)

            path.append(ps)

        ps = PoseStamped()
        ps.header.frame_id = 'base_link'
        ps.pose.position.x = gx
        ps.pose.position.y = gy
        ps.pose.orientation.z = math.sin(gyaw * 0.5)
        ps.pose.orientation.w = math.cos(gyaw * 0.5)
        path.append(ps)
        return path

    def transform_path_to_map(self, path_base, tf):
        tyaw = self.quat_to_yaw(tf.transform.rotation)

        out = []
        for ps in path_base:
            x, y = self.transform_point(
                ps.pose.position.x,
                ps.pose.position.y,
                tf
            )

            yaw = self.quat_to_yaw(ps.pose.orientation) + tyaw

            pm = PoseStamped()
            pm.header.frame_id = 'map'
            pm.pose.position.x = x
            pm.pose.position.y = y
            pm.pose.orientation.z = math.sin(yaw * 0.5)
            pm.pose.orientation.w = math.cos(yaw * 0.5)

            out.append(pm)

        return out

    # ----------------------------
    # Callback
    # ----------------------------
    def publish_empty_markers(self):
        xs = self.get_parameter('roi_x').value
        ys = self.get_parameter('roi_y').value
        roi = list(zip(xs, ys))
        ma = MarkerArray()
        mid = 0

        roi_m = Marker()
        roi_m.header.frame_id = 'base_link'
        roi_m.header.stamp = self.get_clock().now().to_msg()
        roi_m.ns = 'roi'
        roi_m.id = mid
        mid += 1
        roi_m.type = Marker.LINE_STRIP
        roi_m.scale.x = 0.01
        roi_m.color.r = roi_m.color.g = 1.0
        roi_m.color.a = 1.0

        for x, y in roi + [roi[0]]:
            pt = Point()
            pt.x = x
            pt.y = y
            roi_m.points.append(pt)

        ma.markers.append(roi_m)
        self.marker_pub.publish(ma)

    def scan_cb(self, msg):
        try:
            tf_base = self.tf_buffer.lookup_transform(
                'base_link',
                msg.header.frame_id,
                rclpy.time.Time()
            )
        except Exception:
            self.publish_empty_markers()
            return

        xs = self.get_parameter('roi_x').value
        ys = self.get_parameter('roi_y').value
        roi = list(zip(xs, ys))

        points = self.scan_to_points_base(msg, tf_base, roi)
        if not points:
            self.publish_empty_markers()
            return

        clusters = self.cluster_points(points)

        centers = []
        for c in clusters:
            cx, cy = self.centroid(c)
            centers.append(self.compensate_center(cx, cy))

        pairs = self.find_pairs(centers)
        if not pairs:
            self.publish_empty_markers()
            return

        pairs.sort(key=lambda p: abs(self.pair_center(p)[0]))

        # -------- PoseArray --------
        pa = PoseArray()
        pa.header.frame_id = 'base_link'
        pa.header.stamp = msg.header.stamp

        for pair in pairs:
            cx, cy = self.pair_center(pair)
            yaw = self.pair_yaw(pair)

            p = Pose()
            p.position.x = cx
            p.position.y = cy
            p.orientation.z = math.sin(yaw * 0.5)
            p.orientation.w = math.cos(yaw * 0.5)
            pa.poses.append(p)

        self.pose_pub.publish(pa)

        # -------- Path --------
        target = pa.poses[0]
        tx = target.position.x
        ty = target.position.y
        tyaw = self.quat_to_yaw(target.orientation)

        y_thresh = self.get_parameter('y_thresh').value
        yaw_thresh = math.radians(
            self.get_parameter('yaw_thresh_deg').value
        )
        step = self.get_parameter('path_step').value

        if abs(ty) > y_thresh or abs(tyaw) > yaw_thresh:
            offset = 0.42
            self.get_logger().info(f'Using large offset for path goal. ty={ty:.3f}, tyaw={math.degrees(tyaw):.1f} deg')
        else:
            offset = 0.325
            self.get_logger().info(f'Using normal offset for path goal. tx={ty:.3f}')

        gx = tx + offset * math.cos(tyaw)
        gy = ty + offset * math.sin(tyaw)

        path_base = self.generate_path_base(gx, gy, tyaw, step)

        try:
            tf_map = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )
        except Exception:
            return

        path_map = self.transform_path_to_map(path_base, tf_map)

        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = msg.header.stamp
        path_msg.poses = path_map

        self.path_pub.publish(path_msg)

        # -------- Marker --------
        ma = MarkerArray()
        mid = 0

        roi_m = Marker()
        roi_m.header.frame_id = 'base_link'
        roi_m.header.stamp = msg.header.stamp
        roi_m.ns = 'roi'
        roi_m.id = mid
        mid += 1
        roi_m.type = Marker.LINE_STRIP
        roi_m.scale.x = 0.01
        roi_m.color.r = roi_m.color.g = 1.0
        roi_m.color.a = 1.0

        for x, y in roi + [roi[0]]:
            pt = Point()
            pt.x = x
            pt.y = y
            roi_m.points.append(pt)

        ma.markers.append(roi_m)

        for pair in pairs:
            for cx, cy in pair:
                m = Marker()
                m.header.frame_id = 'base_link'
                m.header.stamp = msg.header.stamp
                m.ns = 'bottle'
                m.id = mid
                mid += 1
                m.type = Marker.CYLINDER

                m.pose.position.x = cx
                m.pose.position.y = cy
                m.pose.position.z = 0.1
                m.pose.orientation.w = 1.0

                m.scale.x = m.scale.y = 0.064
                m.scale.z = 0.20

                m.color.b = 1.0
                m.color.a = 0.8

                m.lifetime = rclpy.duration.Duration(seconds=0.1).to_msg()

                ma.markers.append(m)

        self.marker_pub.publish(ma)


def main():
    rclpy.init()
    node = BottleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
