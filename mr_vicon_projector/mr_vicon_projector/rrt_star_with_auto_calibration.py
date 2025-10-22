#!/usr/bin/env python3
import math
from typing import Tuple, List, Optional

import json
import importlib.resources as pkgres
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from action_msgs.msg import GoalStatus
from transforms3d.euler import euler2quat

from irobot_create_msgs.action import NavigateToPosition
from visualization_msgs.msg import Marker, MarkerArray

# ------------------ constants / mapping ------------------

SCALE = 2.25  # your Gazebo↔Vicon zoom factor

def swap_scale_gazebo_to_vicon_xy(x_g: float, y_g: float) -> Tuple[float, float]:
    """
    Your fixed Gazebo->Vicon mapping:
      y_v =  x_g / 2.25
      x_v = -y_g / 2.25
    """
    y_v =  x_g / SCALE
    x_v = -y_g / SCALE
    return x_v, y_v

# ------------------ obstacle loading (fixed) ------------------

def load_obstacles_from_pkg() -> List[dict]:
    """
    Read rrt_planner/data/obstacles.json from the installed package.
    This avoids the old no-arg signature problem.
    """
    with pkgres.files('rrt_planner.data').joinpath('obstacles.json').open('r') as f:
        return json.load(f)

# ------------------ a simple RRT* implementation ------------------

class RRTNode:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y
        self.parent: Optional['RRTNode'] = None
        self.cost: float = 0.0

def _dist(a: RRTNode, b: RRTNode) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)

def _collision_free(p: RRTNode, obstacles: List[dict], robot_radius=0.35) -> bool:
    for obs in obstacles:
        ox = obs['pose']['x']
        oy = obs['pose']['y']
        if obs['type'] in ('sphere', 'cylinder'):
            if math.hypot(p.x - ox, p.y - oy) <= obs['radius'] + robot_radius:
                return False
        elif obs['type'] == 'box':
            sx, sy, *_ = obs['size']
            half_x = sx/2 + robot_radius
            half_y = sy/2 + robot_radius
            if (ox - half_x <= p.x <= ox + half_x and
                oy - half_y <= p.y <= oy + half_y):
                return False
    return True

def _nearest(tree: List[RRTNode], rnd: RRTNode) -> RRTNode:
    return min(tree, key=lambda n: _dist(n, rnd))

def _steer(from_node: RRTNode, to_node: RRTNode, step: float) -> RRTNode:
    d = _dist(from_node, to_node)
    if d < step:
        n = RRTNode(to_node.x, to_node.y)
    else:
        th = math.atan2(to_node.y - from_node.y, to_node.x - from_node.x)
        n = RRTNode(from_node.x + step * math.cos(th),
                    from_node.y + step * math.sin(th))
    n.parent = from_node
    n.cost = from_node.cost + _dist(from_node, n)
    return n

def _near(tree: List[RRTNode], node: RRTNode, radius: float) -> List[RRTNode]:
    return [n for n in tree if _dist(n, node) <= radius]

def _extract_path(goal: RRTNode) -> List[Tuple[float, float]]:
    path = []
    cur = goal
    while cur:
        path.append((cur.x, cur.y))
        cur = cur.parent
    return list(reversed(path))

def rrt_star(start: Tuple[float, float],
             goal: Tuple[float, float],
             bounds: dict,
             obstacles: List[dict],
             max_iter=1000,
             step=0.5,
             rewire_radius=1.0,
             goal_tolerance=0.5) -> Optional[List[Tuple[float, float]]]:

    tree: List[RRTNode] = [RRTNode(*start)]
    for _ in range(max_iter):
        rnd = RRTNode(
            np.random.uniform(*bounds['x']),
            np.random.uniform(*bounds['y'])
        )
        nearest = _nearest(tree, rnd)
        new = _steer(nearest, rnd, step)
        if not _collision_free(new, obstacles):
            continue

        # choose best parent from neighbors
        neighbors = _near(tree, new, rewire_radius)
        best_parent = nearest
        best_cost = nearest.cost + _dist(nearest, new)
        for nb in neighbors:
            if _collision_free(RRTNode(nb.x + (new.x - nb.x), nb.y + (new.y - nb.y)), obstacles):
                c = nb.cost + _dist(nb, new)
                if c < best_cost:
                    best_cost = c
                    best_parent = nb
        new.parent = best_parent
        new.cost = best_cost
        tree.append(new)

        # rewire neighbors
        for nb in neighbors:
            if nb is best_parent:
                continue
            cand_cost = new.cost + _dist(new, nb)
            if cand_cost + 1e-9 < nb.cost and _collision_free(nb, obstacles):
                nb.parent = new
                nb.cost = cand_cost

        # goal check
        if _dist(new, RRTNode(*goal)) <= goal_tolerance:
            goal_node = RRTNode(*goal)
            goal_node.parent = new
            goal_node.cost = new.cost + _dist(new, goal_node)
            return _extract_path(goal_node)
    return None

# ------------------ calibration helper ------------------

class ViconCalibrator(Node):
    """Two-click calibration: capture (x1,y1), then move forward to get heading θ."""
    def __init__(self, vicon_topic: str):
        super().__init__('vicon_calibrator_star')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.sub = self.create_subscription(PoseStamped, vicon_topic, self._cb, qos)
        self.latest: Optional[PoseStamped] = None

    def _cb(self, msg: PoseStamped):
        self.latest = msg

    def _wait_for_pose(self):
        while rclpy.ok() and self.latest is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.latest.pose

    def run_interactive(self):
        print("\nVicon Calibration")
        print("1) Place robot at START, reset odom (or /reset_pose), press ENTER…")
        input()
        self.latest = None
        p1 = self._wait_for_pose()
        x1, y1 = p1.position.x, p1.position.y
        print(f"   Reference captured: ({x1:.3f}, {y1:.3f})")

        print("2) Drive robot straight forward a short distance, press ENTER…")
        input()
        self.latest = None
        p2 = self._wait_for_pose()
        x2, y2 = p2.position.x, p2.position.y
        print(f"   Forward sample:     ({x2:.3f}, {y2:.3f})")

        theta = math.atan2(y2 - y1, x2 - x1)
        print(f"Computed heading θ={theta:.4f} rad ({math.degrees(theta):.2f}°)")

        # T_v_from_r: Robot->Vicon (rotation by θ, translate by (x1,y1))
        c, s = math.cos(theta), math.sin(theta)
        T_v_from_r = np.array([
            [c, -s, 0.0, x1],
            [s,  c, 0.0, y1],
            [0,  0, 1.0, 0.0],
            [0,  0, 0.0, 1.0],
        ])
        return T_v_from_r, theta

# ------------------ main node ------------------

class RRTStarWithCalibration(Node):
    def __init__(self):
        super().__init__('rrt_star_with_calibration')

        # 1) Calibration
        self.vicon_topic = '/vrpn_mocap/iRobot_create3_te/pose'
        self.calib = ViconCalibrator(self.vicon_topic)
        T_v_from_r, self.theta = self.calib.run_interactive()
        self.T_r_from_v = np.linalg.inv(T_v_from_r)

        # 2) Subscriptions / publishers
        qos_gazebo = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.create_subscription(PoseStamped, '/gazebo_robot_pose',
                                 self._gazebo_pose_cb, qos_gazebo)

        self.nav_client = ActionClient(self, NavigateToPosition, '/navigate_to_position')

        self.path_pub = self.create_publisher(Path, '/robot_traveled_path', 10)
        self.rrt_path_pub = self.create_publisher(Path, '/rrt_planned_path', 10)

        # thick lines + obstacles
        self.traveled_marker_pub = self.create_publisher(Marker, '/traveled_marker', 10)
        self.planned_marker_pub  = self.create_publisher(Marker, '/planned_marker', 10)
        self.obstacles_pub       = self.create_publisher(MarkerArray, '/obstacles_markers', 10)

        # appearance
        self.line_width_m = 0.05
        self.col_traveled = (1.0, 0.2, 0.2, 1.0)  # red-ish
        self.col_planned  = (0.2, 1.0, 0.2, 1.0)  # Green
        self.col_obstacle = (0.2, 0.8, 1.0, 0.5)  # cyan, semi-transparent

        # state
        self.robot_path = Path()
        self.robot_path.header.frame_id = 'odom'
        self.map_bounds = {'x': (-10, 10), 'y': (-10, 10)}  # adjust as you wish
        self.goal_g = (3.0, 1.0)  # goal in Gazebo/odom
        self.obstacles = load_obstacles_from_pkg()  # <-- FIXED loader

        self.current_pose_ps: Optional[PoseStamped] = None
        self.path_g: List[Tuple[float, float]] = []
        self.path_index = 0
        self.started = False
        self.sending = False
        self.reached = False

        # publish obstacle markers once
        self._publish_obstacles_markers()

        print("\nCalibration complete. RRT* planner is ready.\n")

    # ----- transforms -----
    def odom_to_robot(self, x_g: float, y_g: float) -> Tuple[float, float]:
        x_v, y_v = swap_scale_gazebo_to_vicon_xy(x_g, y_g)
        vec = np.array([[x_v], [y_v], [0.0], [1.0]])
        r = self.T_r_from_v @ vec
        return float(r[0]), float(r[1])

    # ----- callbacks -----
    def _gazebo_pose_cb(self, msg: PoseStamped):
        self.current_pose_ps = msg

        # record path
        if self.started and not self.reached:
            p = PoseStamped()
            p.header.frame_id = 'odom'
            p.header.stamp = self.get_clock().now().to_msg()
            p.pose = msg.pose
            self.robot_path.poses.append(p)
            self.path_pub.publish(self.robot_path)

            trav_m = self._path_to_marker(self.robot_path, self.col_traveled, ns="traveled", marker_id=0)
            self.traveled_marker_pub.publish(trav_m)

        # plan once
        if not self.started:
            sx, sy = msg.pose.position.x, msg.pose.position.y
            self.get_logger().info(f"Start (Gazebo): ({sx:.2f}, {sy:.2f}) → RRT* planning…")

            path = rrt_star((sx, sy), self.goal_g, self.map_bounds, self.obstacles,
                            max_iter=1500, step=0.5, rewire_radius=1.0, goal_tolerance=0.5)
            if not path:
                self.get_logger().error("RRT* failed to find a path.")
                return

            self.path_g = path

            # publish planned path (odom)
            rrt_msg = Path()
            rrt_msg.header.frame_id = 'odom'
            now = self.get_clock().now().to_msg()
            for xg, yg in self.path_g:
                ps = PoseStamped()
                ps.header.frame_id = 'odom'
                ps.header.stamp = now
                ps.pose.position.x = xg
                ps.pose.position.y = yg
                ps.pose.orientation.w = 1.0
                rrt_msg.poses.append(ps)
            self.rrt_path_pub.publish(rrt_msg)

            planned_m = self._path_to_marker(rrt_msg, self.col_planned, ns="planned", marker_id=0)
            self.planned_marker_pub.publish(planned_m)

            self.get_logger().info(f"RRT* produced {len(self.path_g)} waypoints.")
            self.started = True
            self._waypoint_tick()

    # ----- waypoint driving -----
    def _waypoint_tick(self):
        while not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Waiting for /navigate_to_position …")

        if (not self.started or self.reached or self.sending or
            self.path_index >= len(self.path_g)):
            return

        xg, yg = self.path_g[self.path_index]
        xr, yr = self.odom_to_robot(xg, yg)
        self.get_logger().info(f"[{self.path_index+1}/{len(self.path_g)}] Gazebo→Robot: ({xg:.2f},{yg:.2f}) → ({xr:.2f},{yr:.2f})")

        if self.current_pose_ps:
            dx = xr - self.current_pose_ps.pose.position.x
            dy = yr - self.current_pose_ps.pose.position.y
            if math.hypot(dx, dy) < 0.10:
                self.path_index += 1
                self._waypoint_tick()
                return

        if self.path_index < len(self.path_g) - 1:
            nxg, nyg = self.path_g[self.path_index + 1]
            nxr, nyr = self.odom_to_robot(nxg, nyg)
            yaw = math.atan2(nyr - yr, nxr - xr)
        else:
            yaw = 0.0

        qw, qx, qy, qz = euler2quat(0, 0, yaw)
        goal = NavigateToPosition.Goal()
        goal.goal_pose.header.frame_id = 'odom'
        goal.goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal.goal_pose.pose.position.x = xr
        goal.goal_pose.pose.position.y = yr
        goal.goal_pose.pose.orientation.x = qx
        goal.goal_pose.pose.orientation.y = qy
        goal.goal_pose.pose.orientation.z = qz
        goal.goal_pose.pose.orientation.w = qw
        goal.achieve_goal_heading = False

        self.sending = True
        self.nav_client.send_goal_async(goal).add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().warn('Goal rejected.')
            self.sending = False
            return
        self.get_logger().info('Goal accepted—waiting result…')
        gh.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Waypoint reached.')
        else:
            self.get_logger().warn(f'Waypoint failed (status={status}).')

        self.path_index += 1
        self.sending = False

        if self.path_index >= len(self.path_g):
            self.get_logger().info('Final goal reached. 🎯')
            self.reached = True
        else:
            self._waypoint_tick()

    # ----- markers -----
    def _path_to_marker(self, path_msg: Path, color_rgba, ns: str, marker_id: int) -> Marker:
        r, g, b, a = color_rgba
        m = Marker()
        m.header.frame_id = path_msg.header.frame_id or "odom"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = marker_id
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = self.line_width_m
        m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, a
        m.pose.orientation.w = 1.0
        m.lifetime.sec = 0
        m.points = [p.pose.position for p in path_msg.poses]
        return m

    def _publish_obstacles_markers(self):
        marr = MarkerArray()
        now = self.get_clock().now().to_msg()
        idx = 0
        for obs in self.obstacles:
            m = Marker()
            m.header.frame_id = "odom"
            m.header.stamp = now
            m.ns = "obstacles"
            m.id = idx
            idx += 1
            m.action = Marker.ADD
            m.color.r, m.color.g, m.color.b, m.color.a = self.col_obstacle
            m.pose.position.x = obs["pose"]["x"]
            m.pose.position.y = obs["pose"]["y"]
            m.pose.position.z = 0.0
            m.pose.orientation.w = 1.0

            if obs["type"] in ("sphere", "cylinder"):
                m.type = Marker.CYLINDER
                r = float(obs["radius"])
                m.scale.x = 2*r
                m.scale.y = 2*r
                m.scale.z = 0.02
            elif obs["type"] == "box":
                m.type = Marker.CUBE
                sx, sy, *rest = obs["size"]
                m.scale.x = float(sx)
                m.scale.y = float(sy)
                m.scale.z = 0.02
            else:
                continue

            marr.markers.append(m)

        self.obstacles_pub.publish(marr)

# ------------------ entry point ------------------

def main(args=None):
    rclpy.init(args=args)
    node = RRTStarWithCalibration()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        node.calib.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()