#!/usr/bin/env python3
import math
import csv
import numpy as np
import casadi as ca
from pathlib import Path


class SpeedPathTimeOptimalPlanner:

    def __init__(self):

        # =============================
        # Robot geometry
        # =============================
        self.robot_radius = 0.37
        self.margin = 0.02

        # =============================
        # wheel geometry
        # =============================
        self.wheel_positions = [
            ( 0.173,  0.173),
            (-0.173,  0.173),
            (-0.173, -0.173),
            ( 0.173, -0.173),
        ]
        self.r_max = max(math.hypot(x, y) for x, y in self.wheel_positions)

        # =============================
        # limits
        # =============================
        self.v_wheel_max = 3.0
        self.max_w  = math.pi
        self.max_a  = 5.0
        self.max_aw = 6.28

        # =============================
        # discretization
        # =============================
        self.N = 60

        # =============================
        # waypoints
        # =============================
        self.waypoints = [
            (0.5, 0.5, 0.0),
            (1.0, 3.0, 0.0),
            (1.5, 0.5, 0.1),
        ]

        # =============================
        # map storage
        # =============================
        self.line_segments = []

        self.load_line_segments(
            "../map/line_segments.csv"
        )


    # ============================================================
    # CSV map loader
    # ============================================================
    def load_line_segments(self, path):

        path = Path(path)

        if not path.exists():
            raise FileNotFoundError(f"{path} not found")

        with open(path, newline="") as f:
            reader = csv.reader(f)
            next(reader)  # skip header

            for row in reader:
                x1, y1, z1, x2, y2, z2 = map(float, row)
                self.line_segments.append((x1, y1, x2, y2))

        print(f"Loaded {len(self.line_segments)} line segments")


    # ============================================================
    # Nonlinear optimization
    # ============================================================
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

        # ================= Dynamics =================
        for k in range(N - 1):
            opti.subject_to(x[k+1]   == x[k]   + vx[k] * dt)
            opti.subject_to(y[k+1]   == y[k]   + vy[k] * dt)
            opti.subject_to(yaw[k+1] == yaw[k] + w[k]  * dt)

            opti.subject_to(vx[k+1] == vx[k] + ax[k] * dt)
            opti.subject_to(vy[k+1] == vy[k] + ay[k] * dt)
            opti.subject_to(w[k+1]  == w[k]  + aw[k] * dt)

        # ================= Boundary =================
        opti.subject_to(vx[0] == 0)
        opti.subject_to(vy[0] == 0)
        opti.subject_to(w[0]  == 0)

        opti.subject_to(vx[-1] == 0)
        opti.subject_to(vy[-1] == 0)
        opti.subject_to(w[-1]  == 0)

        # ================= Limits =================
        opti.subject_to(opti.bounded(0.1, T, 20.0))
        opti.subject_to(opti.bounded(-self.max_w, w, self.max_w))
        opti.subject_to(opti.bounded(-self.max_a, ax, self.max_a))
        opti.subject_to(opti.bounded(-self.max_a, ay, self.max_a))
        opti.subject_to(opti.bounded(-self.max_aw, aw, self.max_aw))

        # ================= Wheel constraint =================
        for k in range(N):
            v_trans_sq = vx[k]**2 + vy[k]**2
            v_rot = ca.fabs(w[k]) * self.r_max
            opti.subject_to(
                v_trans_sq <= (self.v_wheel_max - v_rot)**2
            )

        # ================= Obstacle cost =================
        obs_cost = 0
        W_OBS = 1e-2
        safe_r = self.robot_radius + self.margin

        for k in range(N):
            for (x1, y1, x2, y2) in self.line_segments:

                APx = x[k] - x1
                APy = y[k] - y1
                ABx = x2 - x1
                ABy = y2 - y1

                denom = ABx*ABx + ABy*ABy + 1e-6

                t = (APx*ABx + APy*ABy) / denom
                t = ca.fmin(1.0, ca.fmax(0.0, t))

                Qx = x1 + t * ABx
                Qy = y1 + t * ABy

                dx = x[k] - Qx
                dy = y[k] - Qy

                dist_sq = dx*dx + dy*dy
                safe_sq = safe_r**2

                clearance = dist_sq - safe_sq
                obs_cost += W_OBS * ca.fmax(0, -clearance)**2

        # ================= Waypoint cost =================
        wp_cost = 0
        wp_idx = np.linspace(0, N - 1, len(self.waypoints)).astype(int)

        W_POS = 1e4
        W_YAW = 1e2

        for i, (wx, wy, wyaw) in enumerate(self.waypoints):
            k = wp_idx[i]
            wp_cost += W_POS * ((x[k] - wx)**2 + (y[k] - wy)**2)
            angle_error = ca.atan2(
                ca.sin(yaw[k] - wyaw),
                ca.cos(yaw[k] - wyaw)
            )
            wp_cost += W_YAW * angle_error**2

        opti.minimize(T + wp_cost + obs_cost)

        # ================= Initial guess =================
        opti.set_initial(T, 5.0)
        opti.set_initial(x, np.linspace(0, self.waypoints[-1][0], N))
        opti.set_initial(y, np.linspace(0, self.waypoints[-1][1], N))
        opti.set_initial(yaw, np.linspace(0, self.waypoints[-1][2], N))

        opti.solver("ipopt", {
            "ipopt.max_iter": 3000,
            "ipopt.tol": 1e-4,
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


    # ============================================================
    # CSV output
    # ============================================================
    def save_csv(self, filename, T, x, y, yaw, vx, vy, w):

        dt = T / (self.N - 1)

        with open(filename, "w", newline="") as f:
            writer = csv.writer(f)

            writer.writerow([
                "t", "x", "y", "yaw",
                "vx", "vy", "w"
            ])

            for i in range(self.N):
                writer.writerow([
                    i * dt,
                    x[i],
                    y[i],
                    yaw[i],
                    vx[i],
                    vy[i],
                    w[i],
                ])

        print(f"Saved trajectory to {filename}")


# ============================================================
# main
# ============================================================
if __name__ == "__main__":

    planner = SpeedPathTimeOptimalPlanner()

    print("Solving trajectory...")
    T, x, y, yaw, vx, vy, w = planner.solve()

    print(f"Optimal time = {T:.2f} s")

    planner.save_csv(
        "trajectory.csv",
        T, x, y, yaw, vx, vy, w
    )