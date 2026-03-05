import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
import csv
from pathlib import Path
import math

# ===============================
# Parameters
# ===============================

N = 50
W_POS = 1e1
W_YAW = 1e1
W_LEN = 1

max_acc = 3.0
max_vel = 4.0
max_w = np.pi
max_aw = np.pi/ 10

waypoints = []

margin = 0.02
debug_robot_radius = 0.37



while True:
    line = input("x y yaw >>> ").strip()
    if line == "":
        break
    parts = line.split()
    if len(parts) < 2:
        print("Error: x y は必須です")
        continue
    try:
        x = float(parts[0])
        y = float(parts[1])
        
        if len(parts) >= 3:
            yaw_deg = float(parts[2])
            yaw = math.radians(yaw_deg)
        else:
            yaw = 0.0
        
        waypoints.append((x, y, yaw))
    
    except ValueError:
        print("Error: 数値で入力してください")
        continue

print("\nSaved waypoints (rad):")
for wp in waypoints:
    print(wp)

filename = input("\n filename >>> ")
if filename == "":
    filename = "trajectory.csv"

# ===============================
# Load obstacle line segments
# ===============================

line_segments = []

def load_line_segments(path):
    path = Path(path)
    with open(path, newline="") as f:
        reader = csv.reader(f)
        next(reader)
        for row in reader:
            x1, y1, z1, x2, y2, z2 = map(float, row)
            line_segments.append((x1, y1, x2, y2))

load_line_segments("../src/launch/krb2026b_launch/map/line_segments.csv")

# ===============================
# CasADi
# ===============================

opti = ca.Opti()

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

# ===============================
# Dynamics
# ===============================

for k in range(N-1):
    opti.subject_to(x[k+1]  == x[k]  + dt * vx[k] * ca.cos(yaw[k]) - dt * vy[k] * ca.sin(yaw[k]))
    opti.subject_to(y[k+1]  == y[k]  + dt * vx[k] * ca.sin(yaw[k]) + dt * vy[k] * ca.cos(yaw[k]))
    opti.subject_to(yaw[k+1] == yaw[k] + dt * w[k])
    opti.subject_to(vx[k+1] == vx[k] + dt * ax[k])
    opti.subject_to(vy[k+1] == vy[k] + dt * ay[k])
    opti.subject_to(w[k+1]  == w[k]  + dt * aw[k])

# ===============================
# Boundary
# ===============================

opti.subject_to(x[0] == waypoints[0][0])
opti.subject_to(y[0] == waypoints[0][1])
opti.subject_to(vx[0] == 0)
opti.subject_to(vy[0] == 0)
opti.subject_to(w[0] == 0)

opti.subject_to(x[-1] == waypoints[-1][0])
opti.subject_to(y[-1] == waypoints[-1][1])
opti.subject_to(vx[-1] == 0)
opti.subject_to(vy[-1] == 0)
opti.subject_to(w[-1] == 0)

# ===============================
# Limits
# ===============================
opti.subject_to(opti.bounded(0.1, T, 20.0))
opti.subject_to(opti.bounded(-max_acc, ax, max_acc))
opti.subject_to(opti.bounded(-max_acc, ay, max_acc))
opti.subject_to(opti.bounded(-max_aw, aw, max_aw))
opti.subject_to(opti.bounded(-max_vel, vx, max_vel))
opti.subject_to(opti.bounded(-max_vel, vy, max_vel))
opti.subject_to(opti.bounded(-max_w, w, max_w))


# ===============================
# Obstacle soft cost
# ===============================

def point_to_segment_distance_sqrt(px, py, x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    l2 = dx*dx + dy*dy + 1e-8
    t = ((px-x1)*dx + (py-y1)*dy) / l2
    t = ca.fmin(1.0, ca.fmax(0.0, t))
    proj_x = x1 + t*dx
    proj_y = y1 + t*dy
    return (px-proj_x)**2 + (py-proj_y)**2

# ===============================
# Obstacle hard constraints
# ===============================
safe_r = debug_robot_radius + margin
safe_sq = safe_r**2

for k in range(N):
    for (x1, y1, x2, y2) in line_segments:

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

        # ハード制約
        opti.subject_to(dist_sq - safe_sq >= 0)

# ===============================
# Cost
# ===============================
wp_cost = 0
wp_idx = np.linspace(0, N - 1, len(waypoints)).astype(int)
for i, (wx, wy, wyaw) in enumerate(waypoints):
    idx = wp_idx[i]
    wp_cost += W_POS * ((x[idx] - wx)**2 + (y[idx] - wy)**2)
    angle_error = ca.atan2(ca.sin(yaw[idx] - wyaw), ca.cos(yaw[idx] - wyaw))
    wp_cost += W_YAW * angle_error**2

path_length = 0
for k in range(N-1):
    dx = x[k+1] - x[k]
    dy = y[k+1] - y[k]
    path_length += ca.sqrt(dx*dx + dy*dy + 1e-6)

length_cost = W_LEN * path_length

# ===============================
# Initial guess
# ===============================
opti.set_initial(T, 5.0)
x_init = np.zeros(N)
y_init = np.zeros(N)
yaw_init = np.zeros(N)

for i in range(len(waypoints)-1):
    k0 = wp_idx[i]
    k1 = wp_idx[i+1]

    x0, y0, yaw0 = waypoints[i]
    x1, y1, yaw1 = waypoints[i+1]

    x_init[k0:k1+1] = np.linspace(x0, x1, k1-k0+1)
    y_init[k0:k1+1] = np.linspace(y0, y1, k1-k0+1)
    yaw_init[k0:k1+1] = np.linspace(yaw0, yaw1, k1-k0+1)

opti.set_initial(x, x_init)
opti.set_initial(y, y_init)
opti.set_initial(yaw, yaw_init)

# ===============================
# Solver
# ===============================

opti.solver("ipopt",{
    "ipopt.print_level":0,
    "ipopt.max_iter":3000
})

opti.minimize(T + wp_cost + length_cost)
sol = opti.solve()

print(f"Optimal time: {sol.value(T):.2f} sec")

dt = sol.value(T) / (N - 1)

with open(filename, "w", newline="") as f:
    writer = csv.writer(f)

    writer.writerow([
        "t", "x", "y", "yaw",
        "vx", "vy", "w"
    ])

    for i in range(N):
        writer.writerow([
            i * dt,
            sol.value(x[i]),
            sol.value(y[i]),
            sol.value(yaw[i]),
            sol.value(vx[i]),
            sol.value(vy[i]),
            sol.value(w[i]),
        ])

print(f"Saved trajectory to {filename}")



# ===============================
# Visualization
# ===============================

x_vals = np.array([sol.value(x[i]) for i in range(N)])
y_vals = np.array([sol.value(y[i]) for i in range(N)])
yaw_vals = np.array([sol.value(yaw[i]) for i in range(N)])

vx_vals = np.array([sol.value(vx[i]) for i in range(N)])
vy_vals = np.array([sol.value(vy[i]) for i in range(N)])
w_vals  = np.array([sol.value(w[i])  for i in range(N)])

ax_vals = np.array([sol.value(ax[i]) for i in range(N)])
ay_vals = np.array([sol.value(ay[i]) for i in range(N)])
aw_vals = np.array([sol.value(aw[i]) for i in range(N)])

t_vals = np.linspace(0, sol.value(T), N)

# ===============================
# Figure layout
# ===============================

fig, axs = plt.subplots(2,2, figsize=(13,10))

ax_traj = axs[0,0]
ax_x    = axs[0,1]
ax_y    = axs[1,0]
ax_yaw  = axs[1,1]

# ===============================
# Trajectory
# ===============================

collision_flags = []

for i in range(N):

    px = x_vals[i]
    py = y_vals[i]

    collided = False

    for seg in line_segments:

        x1,y1,x2,y2 = seg

        dx = x2-x1
        dy = y2-y1
        l2 = dx*dx + dy*dy + 1e-8

        t = ((px-x1)*dx + (py-y1)*dy)/l2
        t = max(0, min(1, t))

        proj_x = x1 + t*dx
        proj_y = y1 + t*dy

        dist = np.sqrt((px-proj_x)**2 + (py-proj_y)**2)

        if dist < debug_robot_radius + margin:
            collided = True
            break

    collision_flags.append(collided)

for k in range(N-1):

    if collision_flags[k]:
        ax_traj.plot(x_vals[k:k+2], y_vals[k:k+2],'r')
    else:
        ax_traj.plot(x_vals[k:k+2], y_vals[k:k+2],'b')

# waypoint

ax_traj.scatter(
    [w[0] for w in waypoints],
    [w[1] for w in waypoints]
)

# obstacle

for seg in line_segments:
    ax_traj.plot([seg[0],seg[2]],[seg[1],seg[3]],'k')

# heading

arrow_len = 0.25

for i in range(N):

    dx = arrow_len*np.cos(yaw_vals[i])
    dy = arrow_len*np.sin(yaw_vals[i])

    ax_traj.arrow(
        x_vals[i],
        y_vals[i],
        dx,
        dy,
        head_width=0.05,
        head_length=0.07
    )

ax_traj.set_aspect("equal")
ax_traj.set_title("Trajectory")
ax_traj.grid()

# ===============================
# x-t graph
# ===============================

ax_x.plot(t_vals, x_vals, label="x")
ax_x.plot(t_vals, vx_vals, label="vx")
ax_x.plot(t_vals, ax_vals, label="ax")

ax_x.set_title("X state")
ax_x.set_xlabel("time")
ax_x.legend()
ax_x.grid()

# ===============================
# y-t graph
# ===============================

ax_y.plot(t_vals, y_vals, label="y")
ax_y.plot(t_vals, vy_vals, label="vy")
ax_y.plot(t_vals, ay_vals, label="ay")

ax_y.set_title("Y state")
ax_y.set_xlabel("time")
ax_y.legend()
ax_y.grid()

# ===============================
# yaw graph
# ===============================

ax_yaw.plot(t_vals, yaw_vals, label="yaw")
ax_yaw.plot(t_vals, w_vals, label="w")
ax_yaw.plot(t_vals, aw_vals, label="aw")

ax_yaw.set_title("Yaw state")
ax_yaw.set_xlabel("time")
ax_yaw.legend()
ax_yaw.grid()

plt.tight_layout()
try:
    plt.show()
except:
    plt.close()