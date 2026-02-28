#!/usr/bin/env python3
# coding: utf-8

import math
import numpy as np
import casadi as ca

import rclpy
from rclpy.node import Node
from natto_msgs.msg import SpeedPath
from geometry_msgs.msg import PoseStamped, TwistStamped, Quaternion


class TimeOptimalOmniPlanner(Node):

    def __init__(self):
        super().__init__("time_optimal_omni_planner")

        # ---------------- geometry ----------------
        wheel_positions = [
            ( 0.173,  0.173),
            (-0.173,  0.173),
            (-0.173, -0.173),
            ( 0.173, -0.173),
        ]
        self.r_max = max(math.hypot(x, y) for x, y in wheel_positions)

        # ---------------- limits ----------------
        self.v_wheel_max = 3.0
        self.max_v = 2.0
        self.max_w = math.pi
        self.max_a = 5.0
        self.max_aw = 6.28

        # derived safe angular limit from wheel constraint
        self.max_w_from_wheel = self.v_wheel_max / self.r_max

        # ---------------- discretization ----------------
        self.N = 80
        self.T_min = 0.2
        self.T_max = 40.0

        # ---------------- waypoints ----------------
        self.waypoints = [
            (0.5, 0.5, 0.0),
            (1.0, 3.0, 1.57),
            (1.5, 0.5, 0.0),
        ]

        self.pub = self.create_publisher(SpeedPath, "/planning/speed_path", 1)
        self.timer = self.create_timer(1.0, self.run_once)
        self.executed = False

    # ==================================================
    # OPTIMIZATION
    # ==================================================

    def solve(self):

        opti = ca.Opti()
        N = self.N

        T = opti.variable()
        dt = T / (N - 1)

        # ----- states -----
        x   = opti.variable(N)
        y   = opti.variable(N)
        yaw = opti.variable(N)

        vx = opti.variable(N)
        vy = opti.variable(N)
        w  = opti.variable(N)

        # ----- controls -----
        ax = opti.variable(N)
        ay = opti.variable(N)
        aw = opti.variable(N)

        # ==================================================
        # Dynamics (multiple shooting Euler)
        # ==================================================

        for k in range(N - 1):

            c = ca.cos(yaw[k])
            s = ca.sin(yaw[k])

            opti.subject_to(
                x[k+1] == x[k] + dt * (c * vx[k] - s * vy[k])
            )

            opti.subject_to(
                y[k+1] == y[k] + dt * (s * vx[k] + c * vy[k])
            )

            opti.subject_to(
                yaw[k+1] == yaw[k] + dt * w[k]
            )

            opti.subject_to(
                vx[k+1] == vx[k] + dt * ax[k]
            )

            opti.subject_to(
                vy[k+1] == vy[k] + dt * ay[k]
            )

            opti.subject_to(
                w[k+1] == w[k] + dt * aw[k]
            )

        # ==================================================
        # Boundary conditions
        # ==================================================

        opti.subject_to(vx[0] == 0)
        opti.subject_to(vy[0] == 0)
        opti.subject_to(w[0]  == 0)

        opti.subject_to(vx[-1] == 0)
        opti.subject_to(vy[-1] == 0)
        opti.subject_to(w[-1]  == 0)

        # ==================================================
        # Waypoints (hard constraints)
        # ==================================================

        wp_idx = np.linspace(0, N - 1, len(self.waypoints)).astype(int)

        for i, (wx, wy, wyaw) in enumerate(self.waypoints):
            k = int(wp_idx[i])
            opti.subject_to(x[k] == wx)
            opti.subject_to(y[k] == wy)
            opti.subject_to(yaw[k] == wyaw)

        # ==================================================
        # Time bounds
        # ==================================================

        opti.subject_to(opti.bounded(self.T_min, T, self.T_max))

        # ==================================================
        # Velocity limits (NO sqrt)
        # ==================================================

        for k in range(N):

            # translational velocity
            opti.subject_to(
                vx[k]**2 + vy[k]**2 <= self.max_v**2
            )

            # angular velocity
            opti.subject_to(
                opti.bounded(
                    -min(self.max_w, self.max_w_from_wheel),
                     w[k],
                     min(self.max_w, self.max_w_from_wheel)
                )
            )

        # ==================================================
        # Acceleration limits (NO sqrt)
        # ==================================================

        for k in range(N):

            opti.subject_to(
                ax[k]**2 + ay[k]**2 <= self.max_a**2
            )

            opti.subject_to(
                opti.bounded(-self.max_aw, aw[k], self.max_aw)
            )

        # ==================================================
        # Wheel coupling constraint (NO abs, NO sqrt)
        # ==================================================

        for k in range(N):

            v_sq = vx[k]**2 + vy[k]**2

            # +w side
            opti.subject_to(
                v_sq <= (self.v_wheel_max - self.r_max * w[k])**2
            )

            # -w side
            opti.subject_to(
                v_sq <= (self.v_wheel_max + self.r_max * w[k])**2
            )

        # ==================================================
        # Objective
        # ==================================================

        reg = 1e-3 * (
            ca.sumsqr(ax) +
            ca.sumsqr(ay) +
            ca.sumsqr(aw)
        )

        opti.minimize(T + reg)

        # ==================================================
        # Initial guess
        # ==================================================

        opti.set_initial(T, 8.0)

        x0 = np.linspace(self.waypoints[0][0],
                         self.waypoints[-1][0], N)
        y0 = np.linspace(self.waypoints[0][1],
                         self.waypoints[-1][1], N)
        yaw0 = np.linspace(self.waypoints[0][2],
                           self.waypoints[-1][2], N)

        opti.set_initial(x, x0)
        opti.set_initial(y, y0)
        opti.set_initial(yaw, yaw0)

        opti.set_initial(vx, 0)
        opti.set_initial(vy, 0)
        opti.set_initial(w,  0)

        opti.set_initial(ax, 0)
        opti.set_initial(ay, 0)
        opti.set_initial(aw, 0)

        # ==================================================
        # Solver options (robust)
        # ==================================================

        p_opts = {"expand": True}

        s_opts = {
            "max_iter": 5000,
            "tol": 1e-6,
            "acceptable_tol": 1e-4,
            "mu_strategy": "adaptive",
            "print_level": 0,
        }

        opti.solver("ipopt", p_opts, s_opts)

        sol = opti.solve()

        # ==================================================
        # Extract
        # ==================================================

        T_opt = float(sol.value(T))
        dt_val = T_opt / (N - 1)

        t = np.arange(N) * dt_val

        return (
            t,
            T_opt,
            sol.value(x),
            sol.value(y),
            sol.value(yaw),
            sol.value(vx),
            sol.value(vy),
            sol.value(w),
        )

    # ==================================================
    # ROS publish
    # ==================================================

    def run_once(self):

        self.get_logger().info("Solving trajectory...")

        try:
            t, T_opt, x, y, yaw, vx, vy, w = self.solve()
        except Exception as e:
            self.get_logger().error(f"Solve failed: {e}")
            self.executed = True
            return

        self.get_logger().info(f"Optimal time = {T_opt:.3f} s")

        msg = SpeedPath()
        msg.header.frame_id = "odom"
        msg.header.stamp = self.get_clock().now().to_msg()

        for i in range(self.N):

            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(x[i])
            pose.pose.position.y = float(y[i])

            pose.pose.orientation = Quaternion(
                x=0.0,
                y=0.0,
                z=math.sin(float(yaw[i]) * 0.5),
                w=math.cos(float(yaw[i]) * 0.5),
            )
            msg.path.append(pose)

            twist = TwistStamped()
            twist.header = msg.header
            twist.twist.linear.x  = float(vx[i])
            twist.twist.linear.y  = float(vy[i])
            twist.twist.angular.z = float(w[i])

            msg.twist.append(twist)

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = TimeOptimalOmniPlanner()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()