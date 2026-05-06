#!/usr/bin/env python3
"""
RAS 598 Assignment 2 - Motion Planning
A* + Line-of-Sight pruning + Turn-Go-Turn controller
"""
import math
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray
from example_interfaces.srv import Trigger
from ament_index_python.packages import get_package_share_directory

from ras598_assignment_2.grid_map import GridMap
from ras598_assignment_2.a_star   import astar

# ── State constants ──────────────────────────────────────────────────
IDLE   = "IDLE"
ROTATE = "ROTATE"
DRIVE  = "DRIVE"
DONE   = "DONE"


# ── Line-of-sight (Bresenham) ─────────────────────────────────────────
def has_line_of_sight(grid, p1, p2):
    rows, cols = grid.shape
    x0, y0 = p1
    x1, y1 = p2
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    while True:
        if not (0 <= x0 < cols and 0 <= y0 < rows):
            return False
        if grid[y0, x0] == 1:
            return False
        if x0 == x1 and y0 == y1:
            return True
        e2 = 2 * err
        if e2 > -dy:
            err -= dy; x0 += sx
        if e2 <  dx:
            err += dx; y0 += sy


def prune_path(grid, path):
    """Remove intermediate waypoints when clear LOS exists."""
    if len(path) < 2:
        return list(path)
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


# ── Planner Node ──────────────────────────────────────────────────────
class PlannerNode(Node):

    # Tunable parameters
    CELL_RESOLUTION  = 0.2
    INFLATION_RADIUS = 0.65   # > 0.6 as required
    HEADING_THRESH   = 0.05   # rad — switch rotate→drive
    REDRIVE_THRESH   = 0.25   # rad — switch drive→rotate
    WAYPOINT_DIST    = 1.00   # m   — accept radius for intermediate WPs
    GOAL_DIST        = 0.48   # m   — accept radius for final goal
    MAX_LINEAR       = 1.85   # m/s
    MAX_ANGULAR      = 2.0    # rad/s
    KP_ANGULAR       = 2.0    # proportional gain for rotation
    CTRL_HZ          = 20     # Hz

    def __init__(self):
        super().__init__("planner_node")

        pkg = get_package_share_directory("ras598_assignment_2")
        self.MAP_IMAGE = os.path.join(pkg, "cave_filled.png")
        self.MAP_YAML  = os.path.join(pkg, "map.yaml")

        # Publishers
        self.cmd_pub    = self.create_publisher(Twist,       "/cmd_vel",         10)
        self.marker_pub = self.create_publisher(MarkerArray, "/planner_markers", 10)

        # Subscribers
        self.create_subscription(Odometry, "/ground_truth",   self.odom_cb,   10)
        self.create_subscription(Float32,  "/energy_consumed", self.energy_cb, 10)

        # State
        self.pose             = None
        self.energy           = 0.0
        self.raw_path_world   = []
        self.pruned_path_world = []
        self.waypoints        = []
        self.wp_idx           = 0
        self.state            = IDLE
        self.task_called      = False

        # Timers
        self.create_timer(1.0 / self.CTRL_HZ, self.control_loop)
        self.create_timer(2.5, self.init_task)   # delay to let scout start
        self.get_logger().info("PlannerNode ready.")

    # ── Task init ────────────────────────────────────────────────────
    def init_task(self):
        if self.task_called:
            return
        self.task_called = True
        client = self.create_client(Trigger, "/get_task")
        self.get_logger().info("Waiting for /get_task ...")
        if not client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("/get_task not available!")
            return
        future = client.call_async(Trigger.Request())
        future.add_done_callback(self.task_cb)

    def task_cb(self, future):
        try:
            result = future.result()
            self.get_logger().info(f"Task: {result.message}")
            self.plan(result.message)
        except Exception as e:
            self.get_logger().error(f"Task failed: {e}")

    # ── Planning ─────────────────────────────────────────────────────
    def plan(self, task_str):
        sx, sy, gx, gy = [float(v) for v in task_str.split(",")]
        self.get_logger().info(f"Start=({sx},{sy})  Goal=({gx},{gy})")

        self.gmap = GridMap(
            self.MAP_IMAGE, self.MAP_YAML,
            cell_resolution=self.CELL_RESOLUTION,
            inflation_radius_m=self.INFLATION_RADIUS)

        start_cell = self.gmap.world_to_cell(sx, sy)
        goal_cell  = self.gmap.world_to_cell(gx, gy)
        self.get_logger().info(f"Grid: start={start_cell}  goal={goal_cell}")

        raw = astar(self.gmap.grid, start_cell, goal_cell)
        if not raw:
            self.get_logger().error("A* found no path!")
            return
        self.get_logger().info(f"A* raw: {len(raw)} cells")

        self.raw_path_world = [self.gmap.cell_to_world(c, r) for c, r in raw]

        pruned = prune_path(self.gmap.grid, raw)
        self.pruned_path_world = [self.gmap.cell_to_world(c, r) for c, r in pruned]
        self.get_logger().info(
            f"Pruned: {len(self.pruned_path_world)} waypoints from {len(raw)}")

        # Exact goal as last waypoint
        self.pruned_path_world[-1] = (gx, gy)
        self.waypoints = self.pruned_path_world[1:]
        self.wp_idx    = 0
        self.state     = ROTATE

    # ── Odometry ─────────────────────────────────────────────────────
    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (x, y, yaw)

    def energy_cb(self, msg):
        self.energy = msg.data

    # ── Control loop ─────────────────────────────────────────────────
    def control_loop(self):
        if self.state in (IDLE, DONE) or self.pose is None:
            return

        rx, ry, ryaw = self.pose

        # Wait until robot is near expected start before executing
        if self.state == ROTATE and self.wp_idx == 0:
            if math.hypot(rx - (-7.0), ry - (-7.0)) > 2.0:
                self.get_logger().warn(
                    f"Waiting for valid pose ({rx:.2f},{ry:.2f})",
                    throttle_duration_sec=1.0)
                return

        if self.wp_idx >= len(self.waypoints):
            self.finish()
            return

        gx, gy = self.waypoints[self.wp_idx]
        dx, dy  = gx - rx, gy - ry
        dist    = math.hypot(dx, dy)
        angle   = math.atan2(dy, dx)
        err     = self._wrap(angle - ryaw)

        is_last = (self.wp_idx == len(self.waypoints) - 1)
        accept  = self.GOAL_DIST if is_last else self.WAYPOINT_DIST

        if dist < accept:
            self.wp_idx += 1
            if self.wp_idx >= len(self.waypoints):
                self.finish()
                return
            gx, gy = self.waypoints[self.wp_idx]
            err = self._wrap(math.atan2(gy - ry, gx - rx) - ryaw)

        cmd = Twist()

        if self.state == ROTATE:
            if abs(err) <= self.HEADING_THRESH:
                self.state = DRIVE
            else:
                w = max(-self.MAX_ANGULAR,
                        min(self.MAX_ANGULAR, self.KP_ANGULAR * err))
                cmd.angular.z = w
                cmd.linear.x  = 0.05   # tiny forward keeps momentum

        if self.state == DRIVE:
            if abs(err) > self.REDRIVE_THRESH:
                self.state = ROTATE
            else:
                scale = max(0.4, 1.0 - abs(err) / self.REDRIVE_THRESH)
                cmd.linear.x  = self.MAX_LINEAR * scale
                cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)
        self.publish_markers()

    def finish(self):
        self.state = DONE
        self.cmd_pub.publish(Twist())
        self.get_logger().info(
            f"DONE! Energy: {self.energy:.4f}  Pose: {self.pose}")
        self.publish_markers()

    # ── Markers ──────────────────────────────────────────────────────
    def publish_markers(self):
        ma = MarkerArray()
        # Green = raw A* path
        ma.markers.append(self._line_strip(
            0, self.raw_path_world,    r=0.0, g=1.0, b=0.0, w=0.05, ns="raw"))
        # Blue = pruned path
        ma.markers.append(self._line_strip(
            1, self.pruned_path_world, r=0.0, g=0.0, b=1.0, w=0.10, ns="pruned"))
        # Red sphere = current target waypoint
        if self.wp_idx < len(self.waypoints):
            wx, wy = self.waypoints[self.wp_idx]
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp    = self.get_clock().now().to_msg()
            m.ns = "goal"; m.id = 2
            m.type   = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = wx
            m.pose.position.y = wy
            m.pose.position.z = 0.1
            m.scale.x = m.scale.y = m.scale.z = 0.35
            m.color.r = 1.0; m.color.a = 1.0
            ma.markers.append(m)
        self.marker_pub.publish(ma)

    def _line_strip(self, mid, points, r, g, b, w, ns):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp    = self.get_clock().now().to_msg()
        m.ns = ns; m.id = mid
        m.type   = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = w
        m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 1.0
        m.pose.orientation.w = 1.0
        for wx, wy in points:
            p = Point(); p.x = wx; p.y = wy; p.z = 0.05
            m.points.append(p)
        return m

    def _wrap(self, a):
        while a >  math.pi: a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
