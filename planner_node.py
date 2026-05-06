#!/usr/bin/env python3
"""
RAS 598 Assignment 2 - Motion Planning
Complete planner: A* + path pruning + Turn-Go-Turn controller
Optimized for low energy: fewer stops, higher speed, proportional control
"""
import math
import heapq
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray
from example_interfaces.srv import Trigger


# ==========================================
# CONFIGURATION
# ==========================================
MAP_PATH         = '/home/eva/ros2_ws/src/ras598_assignment_2/cave_filled.png'
RESOLUTION       = 0.032   # meters per pixel (from map.yaml)
ORIGIN_X         = -8.0    # world X of image bottom-left
ORIGIN_Y         = -8.0    # world Y of image bottom-left
INFLATION_RADIUS = 0.7     # meters - safety buffer around walls
GRID_RESOLUTION  = 0.2     # meters per A* cell

# Controller - OPTIMIZED for low energy
LINEAR_SPEED    = 1.5  # m/s — faster = less base drain time
ANGULAR_SPEED   = 2.0      # rad/s max rotation
KP_ANGULAR      = 2.5      # proportional gain — smooth turns
HEADING_THRESH  = 0.05     # rad — tight threshold to switch to drive
REDRIVE_THRESH  = 0.30    # rad — only re-rotate if very far off
GOAL_THRESH     = 1.0      # m   — accept waypoint early = fewer stops
FINAL_THRESH    = 0.5      # m   — final goal acceptance radius
MINI_LINEAR     = 0.05     # m/s — tiny forward speed while rotating
                            #        = never fully stops = no startup tax!


# ==========================================
# COORDINATE TRANSFORMS
# ==========================================
def world_to_pixel(wx, wy, img_height):
    col = int((wx - ORIGIN_X) / RESOLUTION)
    row = int(img_height - (wy - ORIGIN_Y) / RESOLUTION)
    return row, col


def pixel_to_world(row, col, img_height):
    wx = col * RESOLUTION + ORIGIN_X
    wy = (img_height - row) * RESOLUTION + ORIGIN_Y
    return wx, wy


# ==========================================
# OCCUPANCY GRID BUILDER
# ==========================================
def build_occupancy_grid(map_path):
    """Load cave_filled.png, inflate obstacles, build A* grid."""
    img = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f'Cannot load map: {map_path}')

    img_h, img_w = img.shape

    # Black pixels = obstacles
    obstacle_mask = (img < 128).astype(np.uint8)

    # Inflate obstacles by INFLATION_RADIUS
    inflate_px = int(INFLATION_RADIUS / RESOLUTION)
    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (2 * inflate_px + 1, 2 * inflate_px + 1))
    inflated = cv2.dilate(obstacle_mask, kernel)

    # Downsample to A* grid resolution
    grid_w = int(img_w * RESOLUTION / GRID_RESOLUTION)
    grid_h = int(img_h * RESOLUTION / GRID_RESOLUTION)
    scale  = GRID_RESOLUTION / RESOLUTION

    grid = np.zeros((grid_h, grid_w), dtype=np.uint8)
    for gr in range(grid_h):
        for gc in range(grid_w):
            px0 = min(int(gc * scale),       img_w - 1)
            px1 = min(int((gc + 1) * scale), img_w)
            py0 = min(int(gr * scale),       img_h - 1)
            py1 = min(int((gr + 1) * scale), img_h)
            region = inflated[py0:py1, px0:px1]
            if region.size > 0 and region.max() > 0:
                grid[gr, gc] = 1

    return grid, grid_h, grid_w, img_h


# ==========================================
# A* ALGORITHM
# ==========================================
def heuristic(a, b):
    """Euclidean distance heuristic."""
    return math.hypot(b[0] - a[0], b[1] - a[1])


def astar(grid, start, goal):
    """
    A* on occupancy grid.
    start, goal: (row, col) tuples
    Returns list of (row, col) or None.
    """
    grid_h, grid_w = grid.shape

    def in_bounds(r, c):
        return 0 <= r < grid_h and 0 <= c < grid_w

    def passable(r, c):
        return grid[r, c] == 0

    neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1),
                 (-1,-1), (-1, 1), (1,-1), (1, 1)]
    costs     = [1.0, 1.0, 1.0, 1.0,
                 math.sqrt(2), math.sqrt(2), math.sqrt(2), math.sqrt(2)]

    open_set = []
    heapq.heappush(open_set, (0.0, 0.0, start))
    came_from = {}
    g_score = {start: 0.0}

    while open_set:
        _, _, current = heapq.heappop(open_set)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path

        for (dr, dc), cost in zip(neighbors, costs):
            nr, nc = current[0] + dr, current[1] + dc
            if not in_bounds(nr, nc) or not passable(nr, nc):
                continue
            new_g = g_score[current] + cost
            neighbor = (nr, nc)
            if new_g < g_score.get(neighbor, float('inf')):
                g_score[neighbor] = new_g
                h = heuristic(neighbor, goal)
                heapq.heappush(open_set, (new_g + h, h, neighbor))
                came_from[neighbor] = current

    return None


# ==========================================
# PATH PRUNING (Line of Sight)
# ==========================================
def has_line_of_sight(grid, p1, p2):
    """Bresenham LOS check — True if no obstacles between p1 and p2."""
    r1, c1 = p1
    r2, c2 = p2
    dr = abs(r2 - r1)
    dc = abs(c2 - c1)
    sr = 1 if r2 > r1 else -1
    sc = 1 if c2 > c1 else -1
    err = dr - dc
    r, c = r1, c1
    grid_h, grid_w = grid.shape

    while True:
        if not (0 <= r < grid_h and 0 <= c < grid_w):
            return False
        if grid[r, c] == 1:
            return False
        if r == r2 and c == c2:
            return True
        e2 = 2 * err
        if e2 > -dc:
            err -= dc
            r += sr
        if e2 < dr:
            err += dr
            c += sc


def prune_path(grid, path):
    """Remove intermediate waypoints when clear LOS exists."""
    if len(path) <= 2:
        return path
    pruned = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1:
            if has_line_of_sight(grid, path[i], path[j]):
                break
            j -= 1
        pruned.append(path[j])
        i = j
    return pruned


# ==========================================
# PLANNER NODE
# ==========================================
class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')

        # Robot state
        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0
        self.energy     = 0.0
        self.waypoints  = []
        self.raw_path   = []
        self.current_wp = 0
        self.state      = 'IDLE'

        # Publishers
        self.cmd_pub    = self.create_publisher(Twist,       '/cmd_vel',         10)
        self.marker_pub = self.create_publisher(MarkerArray, '/planner_markers', 10)

        # Subscribers
        self.create_subscription(Odometry, '/ground_truth',   self.odom_cb,   10)
        self.create_subscription(Float32,  '/energy_consumed', self.energy_cb, 10)

        # Build occupancy grid
        self.get_logger().info('Loading map and building occupancy grid...')
        self.grid, self.grid_h, self.grid_w, self.img_h = \
            build_occupancy_grid(MAP_PATH)
        self.get_logger().info(f'Grid size: {self.grid_h} x {self.grid_w}')

        # Get task
        self.task_client = self.create_client(Trigger, '/get_task')
        self.get_logger().info('Waiting for /get_task service...')
        self.task_client.wait_for_service(timeout_sec=10.0)
        self.call_get_task()

        # Control at 20Hz
        self.timer = self.create_timer(0.05, self.control_loop)

    # ------------------------------------------------------------------
    def call_get_task(self):
        req    = Trigger.Request()
        future = self.task_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is None:
            self.get_logger().error('Failed to get task!')
            return
        msg   = future.result().message
        self.get_logger().info(f'Task received: {msg}')
        parts = msg.split(',')
        sx, sy = float(parts[0]), float(parts[1])
        gx, gy = float(parts[2]), float(parts[3])
        self.plan_path(sx, sy, gx, gy)

    # ------------------------------------------------------------------
    def plan_path(self, sx, sy, gx, gy):
        start_grid = self.clamp_grid(self.world_to_grid(sx, sy))
        goal_grid  = self.clamp_grid(self.world_to_grid(gx, gy))
        self.get_logger().info(f'Grid start: {start_grid}  goal: {goal_grid}')

        self.get_logger().info('Running A*...')
        path = astar(self.grid, start_grid, goal_grid)
        if path is None:
            self.get_logger().error('A* found no path!')
            return
        self.get_logger().info(f'A* path: {len(path)} cells')

        self.raw_path = [self.grid_to_world(r, c) for r, c in path]

        pruned = prune_path(self.grid, path)
        self.waypoints = [self.grid_to_world(r, c) for r, c in pruned]
        self.get_logger().info(
            f'Pruned path: {len(self.waypoints)} waypoints')

        self.waypoints[-1] = (gx, gy)
        self.current_wp = 1
        self.state = 'ROTATE'
        self.publish_markers()

    # ------------------------------------------------------------------
    def world_to_grid(self, wx, wy):
        col = int((wx - ORIGIN_X) / GRID_RESOLUTION)
        row = self.grid_h - 1 - int((wy - ORIGIN_Y) / GRID_RESOLUTION)
        return row, col

    def grid_to_world(self, row, col):
        wx = col * GRID_RESOLUTION + ORIGIN_X + GRID_RESOLUTION / 2
        wy = (self.grid_h - 1 - row) * GRID_RESOLUTION + ORIGIN_Y + GRID_RESOLUTION / 2
        return wx, wy

    def clamp_grid(self, cell):
        r, c = cell
        r = max(0, min(self.grid_h - 1, r))
        c = max(0, min(self.grid_w - 1, c))
        return r, c

    # ------------------------------------------------------------------
    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def energy_cb(self, msg):
        self.energy = msg.data

    # ------------------------------------------------------------------
    def control_loop(self):
        """
        Optimized Turn-Go-Turn controller.
        Key optimization: MINI_LINEAR during rotation = never fully stops
        = no startup tax penalty!
        """
        if self.state in ('IDLE', 'DONE'):
            return

        if self.current_wp >= len(self.waypoints):
            self.get_logger().info(
                f'Goal reached! Energy: {self.energy:.2f}')
            self.stop_robot()
            self.state = 'DONE'
            return

        tx, ty = self.waypoints[self.current_wp]
        dx = tx - self.x
        dy = ty - self.y
        dist = math.sqrt(dx * dx + dy * dy)

        is_final = (self.current_wp == len(self.waypoints) - 1)
        thresh   = FINAL_THRESH if is_final else GOAL_THRESH

        if dist < thresh:
            self.current_wp += 1
            if self.current_wp < len(self.waypoints):
                self.state = 'ROTATE'
                self.publish_markers()
            return

        desired_yaw = math.atan2(dy, dx)
        heading_err = self.angle_diff(desired_yaw, self.yaw)

        cmd = Twist()

        if self.state == 'ROTATE':
            if abs(heading_err) < HEADING_THRESH:
                self.state = 'DRIVE'
                cmd.linear.x  = LINEAR_SPEED
                cmd.angular.z = 0.0
            else:
                # KEY: proportional angular + tiny linear = no full stop!
                w = KP_ANGULAR * heading_err
                w = max(-ANGULAR_SPEED, min(ANGULAR_SPEED, w))
                cmd.angular.z = w
                cmd.linear.x  = MINI_LINEAR   # never fully stop!

        elif self.state == 'DRIVE':
            if abs(heading_err) > REDRIVE_THRESH:
                self.state = 'ROTATE'
                cmd.linear.x  = MINI_LINEAR
                cmd.angular.z = 0.0
            else:
                # Scale speed based on heading error
                scale = max(0.4, 1.0 - abs(heading_err) / REDRIVE_THRESH)
                cmd.linear.x  = LINEAR_SPEED * scale
                cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def angle_diff(self, a, b):
        d = a - b
        while d >  math.pi: d -= 2 * math.pi
        while d < -math.pi: d += 2 * math.pi
        return d

    # ------------------------------------------------------------------
    def publish_markers(self):
        ma    = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)

        # Green LINE_STRIP = raw A* path
        if self.raw_path:
            m = Marker()
            m.header.frame_id = 'map'
            m.id = 0
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.05
            m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0
            m.color.a = 0.8
            m.pose.orientation.w = 1.0
            for wx, wy in self.raw_path:
                p = Point(); p.x = wx; p.y = wy; p.z = 0.05
                m.points.append(p)
            ma.markers.append(m)

        # Blue LINE_STRIP = pruned path
        if self.waypoints:
            m = Marker()
            m.header.frame_id = 'map'
            m.id = 1
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.10
            m.color.r = 0.0; m.color.g = 0.0; m.color.b = 1.0
            m.color.a = 0.9
            m.pose.orientation.w = 1.0
            for wx, wy in self.waypoints:
                p = Point(); p.x = wx; p.y = wy; p.z = 0.10
                m.points.append(p)
            ma.markers.append(m)

        # Red SPHERE = current target waypoint
        if self.waypoints and self.current_wp < len(self.waypoints):
            tx, ty = self.waypoints[self.current_wp]
            m = Marker()
            m.header.frame_id = 'map'
            m.id = 2
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = tx
            m.pose.position.y = ty
            m.pose.position.z = 0.3
            m.scale.x = m.scale.y = m.scale.z = 0.35
            m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0
            m.color.a = 1.0
            ma.markers.append(m)

        self.marker_pub.publish(ma)


# ==========================================
# MAIN
# ==========================================
def main():
    rclpy.init()
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()