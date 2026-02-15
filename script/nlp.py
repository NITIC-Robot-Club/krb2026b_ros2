#!/usr/bin/env python3
import math
import numpy as np
import casadi as ca

import rclpy
from rclpy.node import Node

from natto_msgs.msg import SpeedPath
from geometry_msgs.msg import PoseStamped, TwistStamped, Quaternion

class SpeedPathTimeOptimalPlanner(Node):
    def __init__(self):
        super().__init__("speed_path_time_optimal_planner")

        # =====================================
        # hard-coded robot / wheel information
        # =====================================
        self.wheel_positions = [
            ( 0.173,  0.173),
            (-0.173,  0.173),
            (-0.173, -0.173),
            ( 0.173, -0.173),
        ]
        self.r_max = max(math.hypot(x, y) for x, y in self.wheel_positions)

        # =====================================
        # limits
        # =====================================
        self.v_wheel_max = 3.0     # [m/s]
        self.max_w  = math.pi     # [rad/s]
        self.max_a  = 5.0         # [m/s^2]
        self.max_aw = 6.28        # [rad/s^2]

        # =====================================
        # discretization
        # =====================================
        self.N = 60

        # =====================================
        # waypoints (offline)
        # =====================================
        self.waypoints = [
            (0.5, 0.5, 0.0),
            (1.0, 3.0, 0.0),
            (1.5, 0.5, 0.0),
        ]

        # =====================================
        # ROS publisher
        # =====================================
        self.pub = self.create_publisher(SpeedPath, "speed_path", 1)

        self.timer = self.create_timer(1.0, self.run_once)
        self.executed = False

    # --------------------------------------------------
    # nonlinear optimization
    # --------------------------------------------------
    def solve(self):
        opti = ca.Opti()
        N = self.N

        T = opti.variable()
        dt = T / (N - 1)

        x   = opti.variable(N)
        y   = opti.variable(N)
        yaw = opti.variable(N)

        vx = opti.variable(N)
        vy = opti.variable(N)
        w  = opti.variable(N)

        ax = opti.variable(N)
        ay = opti.variable(N)
        aw = opti.variable(N)

        # ================= dynamics =================
        for k in range(N - 1):
            opti.subject_to(x[k+1]   == x[k]   + vx[k] * dt)
            opti.subject_to(y[k+1]   == y[k]   + vy[k] * dt)
            opti.subject_to(yaw[k+1] == yaw[k] + w[k]  * dt)

            opti.subject_to(vx[k+1] == vx[k] + ax[k] * dt)
            opti.subject_to(vy[k+1] == vy[k] + ay[k] * dt)
            opti.subject_to(w[k+1]  == w[k]  + aw[k] * dt)

        # ================= boundary =================
        opti.subject_to(vx[0] == 0)
        opti.subject_to(vy[0] == 0)
        opti.subject_to(w[0]  == 0)

        opti.subject_to(vx[-1] == 0)
        opti.subject_to(vy[-1] == 0)
        opti.subject_to(w[-1]  == 0)

        # ================= limits =================
        opti.subject_to(opti.bounded(0.1, T, 20.0))

        opti.subject_to(opti.bounded(-self.max_w, w, self.max_w))
        opti.subject_to(opti.bounded(-self.max_a, ax, self.max_a))
        opti.subject_to(opti.bounded(-self.max_a, ay, self.max_a))
        opti.subject_to(opti.bounded(-self.max_aw, aw, self.max_aw))

        # ================= wheel velocity coupling =================
        for k in range(N):
            v_trans_sq = vx[k]**2 + vy[k]**2
            v_rot = ca.fabs(w[k]) * self.r_max
            opti.subject_to(
                v_trans_sq <= (self.v_wheel_max - v_rot)**2
            )

        # ================= waypoint soft constraints =================
        wp_cost = 0
        wp_idx = np.linspace(0, N - 1, len(self.waypoints)).astype(int)

        W_POS = 1e4
        W_YAW = 1e2

        for i, (wx, wy, wyaw) in enumerate(self.waypoints):
            k = wp_idx[i]
            wp_cost += W_POS * ((x[k] - wx)**2 + (y[k] - wy)**2)
            wp_cost += W_YAW * (yaw[k] - wyaw)**2

        # ================= objective =================
        opti.minimize(T + wp_cost)

        # ================= initial guess =================
        opti.set_initial(T, 5.0)
        opti.set_initial(x, np.linspace(0, self.waypoints[-1][0], N))
        opti.set_initial(y, np.linspace(0, self.waypoints[-1][1], N))
        opti.set_initial(yaw, 0)

        opti.solver("ipopt", {
            "ipopt.max_iter": 3000,
            "ipopt.tol": 1e-4,
            "ipopt.acceptable_tol": 1e-3,
            "ipopt.print_level": 0,
        })

        sol = opti.solve()

        return (
            sol.value(T),
            sol.value(x),
            sol.value(y),
            sol.value(yaw),
            sol.value(vx),
            sol.value(vy),
            sol.value(w),
        )

    # --------------------------------------------------
    # publish SpeedPath
    # --------------------------------------------------
    def run_once(self):
        if self.executed:
            return
        self.executed = True

        self.get_logger().info("Solving time-optimal trajectory...")
        T, x, y, yaw, vx, vy, w = self.solve()
        self.get_logger().info(f"Optimal time = {T:.2f} s")

        msg = SpeedPath()
        msg.header.frame_id = "base_link"
        msg.header.stamp = self.get_clock().now().to_msg()

        for i in range(self.N):
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(x[i])
            pose.pose.position.y = float(y[i])
            pose.pose.position.z = 0.0
            pose.pose.orientation = Quaternion(
                x=0.0,
                y=0.0,
                z=math.sin(yaw[i] * 0.5),
                w=math.cos(yaw[i] * 0.5),
            )
            msg.path.append(pose)

            twist = TwistStamped()
            twist.header = msg.header
            twist.twist.linear.x  = float(vx[i])
            twist.twist.linear.y  = float(vy[i])
            twist.twist.angular.z = float(w[i])
            msg.twist.append(twist)

        self.pub.publish(msg)
        self.get_logger().info("SpeedPath published")

def main():
    rclpy.init()
    node = SpeedPathTimeOptimalPlanner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()