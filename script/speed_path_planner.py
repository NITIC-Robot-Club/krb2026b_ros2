import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
import csv
from pathlib import Path

# ===============================
# Parameters
# ===============================

N = 100
T_val = 5.0      # 固定時間（まずは固定）
dt = T_val / N

max_acc = 5.0
max_vel = 3.0
robot_radius = 0.4

waypoints = [
    (0.5, 0.5),
    (1.0, 3.0),
    (1.5, 0.5)
]

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

X = opti.variable(4, N+1)   # x,y,vx,vy
U = opti.variable(2, N)     # ax,ay

x  = X[0,:]
y  = X[1,:]
vx = X[2,:]
vy = X[3,:]

ax = U[0,:]
ay = U[1,:]

# ===============================
# Dynamics
# ===============================

for k in range(N):
    opti.subject_to(x[k+1]  == x[k]  + dt * vx[k])
    opti.subject_to(y[k+1]  == y[k]  + dt * vy[k])
    opti.subject_to(vx[k+1] == vx[k] + dt * ax[k])
    opti.subject_to(vy[k+1] == vy[k] + dt * ay[k])

# ===============================
# Boundary
# ===============================

opti.subject_to(x[0] == waypoints[0][0])
opti.subject_to(y[0] == waypoints[0][1])
opti.subject_to(vx[0] == 0)
opti.subject_to(vy[0] == 0)

opti.subject_to(x[-1] == waypoints[-1][0])
opti.subject_to(y[-1] == waypoints[-1][1])
opti.subject_to(vx[-1] == 0)
opti.subject_to(vy[-1] == 0)

# 中間Waypoint
mid = N//2
opti.subject_to(x[mid] == waypoints[1][0])
opti.subject_to(y[mid] == waypoints[1][1])

# ===============================
# Limits（円制約）
# ===============================

for k in range(N):
    opti.subject_to(ax[k]**2 + ay[k]**2 <= max_acc**2)

for k in range(N+1):
    opti.subject_to(vx[k]**2 + vy[k]**2 <= max_vel**2)

# ===============================
# Obstacle soft cost
# ===============================

def point_to_segment_distance(px, py, x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    l2 = dx*dx + dy*dy + 1e-8
    t = ((px-x1)*dx + (py-y1)*dy) / l2
    t = ca.fmin(1.0, ca.fmax(0.0, t))
    proj_x = x1 + t*dx
    proj_y = y1 + t*dy
    return ca.sqrt((px-proj_x)**2 + (py-proj_y)**2)

# ===============================
# Obstacle hard constraints
# ===============================

for seg in line_segments:
    x1,y1,x2,y2 = seg
    for k in range(N+1):
        dist = point_to_segment_distance(x[k], y[k], x1,y1,x2,y2)
        opti.subject_to(dist >= robot_radius)

# ===============================
# Cost
# ===============================

s = opti.variable(N+1)  # スラック変数
opti.subject_to(s >= 0)

for seg in line_segments:
    x1,y1,x2,y2 = seg
    for k in range(N+1):
        dist = point_to_segment_distance(x[k], y[k], x1,y1,x2,y2)
        opti.subject_to(dist + s[k] >= robot_radius)

opti.minimize(0.01*ca.sumsqr(U) + 100*ca.sumsqr(s))

# ===============================
# Initial guess
# ===============================

xs = []
ys = []

segments = len(waypoints)-1
pts = N//segments

for i in range(segments):
    x0,y0 = waypoints[i]
    x1,y1 = waypoints[i+1]
    xs.extend(np.linspace(x0,x1,pts,endpoint=False))
    ys.extend(np.linspace(y0,y1,pts,endpoint=False))

xs.append(waypoints[-1][0])
ys.append(waypoints[-1][1])

opti.set_initial(x,xs)
opti.set_initial(y,ys)
opti.set_initial(vx,0)
opti.set_initial(vy,0)
opti.set_initial(U,0)


# ===============================
# Solver
# ===============================

opti.solver("ipopt",{
    "ipopt.print_level":0,
    "print_time":0,
    "ipopt.max_iter":2000
})

sol = opti.solve()

# ===============================
# Extract
# ===============================

X_sol = sol.value(X)
U_sol = sol.value(U)

# ===============================
# Plot
# ===============================
plt.figure()

# ---- 各ステップが衝突しているか判定 ----
collision_flags = []

for k in range(N+1):
    px = X_sol[0,k]
    py = X_sol[1,k]

    collided = False

    for seg in line_segments:
        x1,y1,x2,y2 = seg

        dx = x2 - x1
        dy = y2 - y1
        l2 = dx*dx + dy*dy + 1e-8

        t = ((px-x1)*dx + (py-y1)*dy)/l2
        t = max(0, min(1, t))

        proj_x = x1 + t*dx
        proj_y = y1 + t*dy

        dist = np.sqrt((px-proj_x)**2 + (py-proj_y)**2)

        if dist < robot_radius:
            collided = True
            break

    collision_flags.append(collided)

# ---- 区間ごとに色を変えて描画 ----
for k in range(N):
    if collision_flags[k]:
        plt.plot(X_sol[0,k:k+2],
                 X_sol[1,k:k+2],
                 'r')
    else:
        plt.plot(X_sol[0,k:k+2],
                 X_sol[1,k:k+2],
                 'b')

# Waypoints
plt.scatter([w[0] for w in waypoints],
            [w[1] for w in waypoints])

# 障害物
for seg in line_segments:
    plt.plot([seg[0],seg[2]],
             [seg[1],seg[3]], 'k')

plt.gca().set_aspect("equal")
plt.title("Trajectory (Red = Collision)")
plt.show()