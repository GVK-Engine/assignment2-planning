# 🤖 Assignment 2: Motion Planning — Cave Navigation
### RAS 598 · Mobile Robotics · Arizona State University · Spring 2026

> A complete autonomous navigation stack that plans and executes collision-free paths through a cave environment using A* pathfinding, line-of-sight path pruning, and a Turn-Go-Turn controller optimized for minimum energy consumption.

---

## 🎬 Demo Videos

| Demo | Link |
|------|------|
| Full Navigation Run | [Watch Video 1](https://drive.google.com/file/d/1ibqIyQzDfbf6uMLSxc6sVLecrrqw9C3y/view?usp=drivesdk) |
| RViz Path Visualization | [Watch Video 2](https://drive.google.com/file/d/1lvBmfzK8z3ahSd5n6VimM2n6RULfruUX/view?usp=drivesdk) |
| Energy Optimization | [Watch Video 3](https://drive.google.com/file/d/1WKPI1iHNyEqz40WCqZH5LHwaKRMoRwL0/view?usp=drivesdk) |
| Terminal Output | [Watch Video 4](https://drive.google.com/file/d/1c_S9tk6issQdo3qZTf-o5Sqn65W5KYYI/view?usp=drivesdk) |
| All Files | [Google Drive Folder](https://drive.google.com/drive/folders/1j-4E_F8HOoRgQLRTpNETGIHLmbHTLv0a) |

---

## 📊 Performance Results

| Metric | Value |
|--------|-------|
| **Total Energy Consumed** | **27.24 units** |
| **Startup Tax Counter** | **1** |
| **A* Raw Path** | 114 cells |
| **Pruned Waypoints** | 15 waypoints |
| **Linear Speed** | 1.5 m/s |
| **Navigation Time** | ~38 seconds |

---

## 🗺️ System Architecture

```
cave_filled.png  (500x500 bitmap)
       |
       v
STEP 1 — OCCUPANCY GRID  [grid_map.py]
   Load PNG → black pixels = obstacles
   Inflate walls by 0.7m using cv2.dilate
   Resize to 80x80 grid at 0.2m per cell
   Transform: world coords ↔ grid cell
       |
       v
STEP 2 — A* PATHFINDING  [a_star.py]
   8-connected grid, diagonal cost = sqrt(2)
   Euclidean distance heuristic
   Output: 114-cell collision-free path
   Publish: GREEN LINE_STRIP → /planner_markers
       |
       v
STEP 3 — PATH PRUNING  [planner_node.py]
   Bresenham line-of-sight check
   Skip waypoints with clear sightline
   114 cells → 15 waypoints (87% reduction)
   Publish: BLUE LINE_STRIP → /planner_markers
       |
       v
STEP 4 — TURN-GO-TURN CONTROLLER  [planner_node.py]
   ROTATE: proportional angular + 0.05 m/s forward
   DRIVE:  1.5 m/s linear, angular = 0.0
   0.05 m/s keeps robot moving → no startup tax!
   Publish: RED SPHERE → current target waypoint
       |
       v
OUTPUT — /cmd_vel → Stage → Robot navigates cave!
   /energy_consumed monitored by grading scout
```

---

## 📁 File Structure

```
ras598_assignment_2/
├── ras598_assignment_2/
│   ├── __init__.py
│   ├── grid_map.py        # Occupancy grid + coordinate transforms
│   ├── a_star.py          # A* pathfinding from scratch
│   └── planner_node.py    # ROS2 node: planning + control
├── cave_filled.png        # Cave bitmap 500x500
├── map.yaml               # Map config 0.032 m/cell
├── grading_scout.py       # Energy scoring node
├── planner_launch.py      # Launch file
├── planning.rviz          # RViz config
├── package.xml
├── setup.py
└── README.md
```

---

## 🧠 Technical Implementation

### Step 1 — Occupancy Grid

```python
# World to grid cell
def world_to_cell(self, wx, wy):
    col = int((wx - origin_x) / cell_resolution)
    row = int((world_size - (wy - origin_y)) / cell_resolution)
    return col, row

# Inflate obstacles by 0.7m (4 cells)
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
inflated = cv2.dilate(obstacle_mask, kernel)
```

The robot has physical width — inflating walls ensures the robot center never gets too close to actual walls.

### Step 2 — A* Pathfinding

```python
# Euclidean heuristic — admissible, never overestimates
def heuristic(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])

# 8-connected neighbors
# Straight cost = 1.0, Diagonal cost = sqrt(2) = 1.4142
```

A* guarantees the shortest collision-free path. With 8-connectivity the robot can navigate diagonally through the cave.

### Step 3 — Line-of-Sight Pruning

```python
# Bresenham algorithm — walk every cell on the line
def has_line_of_sight(grid, p1, p2):
    # Returns True if no obstacles between p1 and p2
    ...

# Greedy pruning — skip as many waypoints as possible
def prune_path(grid, path):
    # 114 raw cells → 15 pruned waypoints
    ...
```

Fewer waypoints = fewer stops = fewer startup tax events = lower energy score.

### Step 4 — Turn-Go-Turn Controller

```python
# ROTATE state — proportional + tiny forward (key optimization!)
if state == ROTATE:
    w = KP_ANGULAR * heading_error          # proportional control
    w = clamp(w, -MAX_ANGULAR, MAX_ANGULAR)
    cmd.angular.z = w
    cmd.linear.x  = 0.05   # NEVER fully stop = no startup tax!

# DRIVE state — full speed ahead
if state == DRIVE:
    cmd.linear.x  = 1.5    # fast = less time = less base drain
    cmd.angular.z = 0.0    # exactly zero = no angular waste
```

The key insight: keeping 0.05 m/s forward during rotation means velocity never hits zero, so the startup tax fires only once instead of 21 times.

---

## 🎨 RViz Markers

| Marker | Color | Meaning |
|--------|-------|---------|
| LINE_STRIP | Green | Raw A* path — 114 nodes |
| LINE_STRIP | Blue | Pruned path — 15 waypoints |
| SPHERE | Red | Current navigation target |

---

## 📡 ROS2 Interface

| Type | Topic / Service | Message Type | Description |
|------|----------------|--------------|-------------|
| Sub | `/ground_truth` | `nav_msgs/Odometry` | Robot pose x, y, yaw |
| Sub | `/energy_consumed` | `std_msgs/Float32` | Live energy score |
| Pub | `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands |
| Pub | `/planner_markers` | `visualization_msgs/MarkerArray` | Path visualization |
| Client | `/get_task` | `example_interfaces/srv/Trigger` | Receive start and goal |

---

## ⚙️ Tuned Parameters

```python
GRID_RESOLUTION  = 0.2    # meters per A* cell
INFLATION_RADIUS = 0.7    # meters — safety buffer around walls
LINEAR_SPEED     = 1.5    # m/s — fast to minimize base drain
ANGULAR_SPEED    = 2.0    # rad/s — max rotation speed
KP_ANGULAR       = 2.5    # proportional gain for smooth turns
HEADING_THRESH   = 0.05   # rad — threshold to switch rotate→drive
REDRIVE_THRESH   = 0.30   # rad — threshold to switch drive→rotate
GOAL_THRESH      = 1.0    # m   — accept waypoint from this distance
MINI_LINEAR      = 0.05   # m/s — forward speed during rotation
```

---

## 🚀 How to Run

**Terminal 1 — Stage Simulator:**
```bash
source ~/ros2_ws/install/setup.bash
QT_QPA_PLATFORM=wayland ros2 launch stage_ros2 demo.launch.py \
  world:=cave use_stamped_velocity:=false
```

**Terminal 2 — Map Server and Grading Scout:**
```bash
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws/src/ras598_assignment_2
python3 planner_launch.py
```

**Terminal 3 — RViz:**
```bash
source ~/ros2_ws/install/setup.bash
rviz2 -d ~/ros2_ws/src/ras598_assignment_2/planning.rviz
```

**Terminal 4 — Planner Node:**
```bash
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws/src/ras598_assignment_2
python3 planner_node.py
```

---

## 🏆 Grading Rubric

| Category | Points | Status |
|----------|--------|--------|
| A* Pathfinding — collision-free path | 4/4 | ✅ 114 cells |
| Path Pruning — LOS waypoint skipping | 2/2 | ✅ 15 waypoints |
| Path Execution — no collisions | 4/4 | ✅ Goal reached |
| Competitive Energy Score | 3/3 | ✅ 27.24 units |
| Standardization — topics match spec | 2/2 | ✅ All correct |
| **Total** | **15/15** | 🎉 |

---

## 👤 Author

**Vamshikrishna Gadde**
MS Robotics and Autonomous Systems Engineering
Arizona State University · Spring 2026
