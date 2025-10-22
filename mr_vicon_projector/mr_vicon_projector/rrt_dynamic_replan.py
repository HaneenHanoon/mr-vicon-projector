#!/usr/bin/env python3
# rrt_dynamic_replan.py
import math
from typing import Tuple, List, Dict, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import Marker, MarkerArray
from action_msgs.msg import GoalStatus
from transforms3d.euler import euler2quat

from irobot_create_msgs.action import NavigateToPosition
from rrt_planner.rrt_planner import rrt, load_obstacles

# ----------------------------
# Tunables
# ----------------------------
SCALE = 2.25                         # Gazebo <-> Vicon scale
LINE_WIDTH_M = 0.06
COL_TRAVELED = (1.0, 0.2, 0.2, 1.0)
COL_PLANNED  = (0.2, 1.0, 0.2, 1.0)
COL_OBS      = (0.2, 0.8, 1.0, 0.8)
FRAME = "odom"

# Dynamic obstacle / replan
TB_NAMES        = ['tb3_0', 'tb3_1', 'tb3_2']  # edit as needed
ROBOT_RADIUS    = 0.35
TB_RADIUS       = 0.20
SAFETY_BUF      = 0.07
CORRIDOR_W      = ROBOT_RADIUS + TB_RADIUS + SAFETY_BUF

PRED_HORIZON_S      = 3.0
PRED_DT_S           = 0.1
REPLAN_COOLDOWN_S   = 0.7
EMERGENCY_RADIUS    = 0.55
DYN_SUBSAMPLE       = 2

# Soft pause/brake
SOFT_BRAKE_S    = 0.7
BRAKE_TTC_S     = 0.7

# TB marker (no path trails)
TB_LABELS       = True        # set False to hide labels
TB_DISC_ALPHA   = 1.0
TB_COLORS       = [
    (0.95, 0.35, 0.35, TB_DISC_ALPHA),
    (0.35, 0.85, 0.35, TB_DISC_ALPHA),
    (0.35, 0.55, 0.95, TB_DISC_ALPHA),
    (0.95, 0.80, 0.35, TB_DISC_ALPHA),
    (0.75, 0.45, 0.95, TB_DISC_ALPHA),
    (0.35, 0.85, 0.85, TB_DISC_ALPHA),
]


# ----------------------------
# Helpers
# ----------------------------
def swap_scale_gazebo_to_vicon_xy(x_g: float, y_g: float) -> Tuple[float, float]:
    y_v =  x_g / SCALE
    x_v = -y_g / SCALE
    return x_v, y_v

def quat_from_yaw(yaw: float) -> Quaternion:
    w, x, y, z = euler2quat(0.0, 0.0, yaw)
    q = Quaternion()
    q.w, q.x, q.y, q.z = w, x, y, z
    return q


# ----------------------------
# Calibration helper
# ----------------------------
class ViconCalibrator(Node):
    """Two-click calibration: capture (x1,y1); move forward; capture (x2,y2)."""
    def __init__(self, vicon_topic: str):
        super().__init__('vicon_calibrator')
        self.pose: Optional[PoseStamped] = None
        self.create_subscription(
            PoseStamped, vicon_topic, self._vicon_cb,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST, depth=1)
        )

    def _vicon_cb(self, msg: PoseStamped):
        self.pose = msg

    def run_interactive(self):
        self.get_logger().info("Vicon Calibration\n1) Place robot at START, reset odom, press ENTER…")
        input()
        while self.pose is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        x1, y1 = self.pose.pose.position.x, self.pose.pose.position.y

        self.get_logger().info("2) Move ~0.5–1.0m *forward* (same heading). Press ENTER…")
        input()
        p2 = None
        while p2 is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            p2 = self.pose
        x2, y2 = p2.pose.position.x, p2.pose.position.y

        dx, dy = x2 - x1, y2 - y1
        theta = math.atan2(dy, dx)

        c, s = math.cos(theta), math.sin(theta)
        T_v_from_r = np.array([
            [c, -s, 0.0, x1],
            [s,  c, 0.0, y1],
            [0,  0, 1.0, 0.0],
            [0,  0, 0.0, 1.0],
        ])
        return T_v_from_r, theta


# ----------------------------
# Main Planner Node
# ----------------------------
class RRTPlannerWithCalibration(Node):
    def __init__(self):
        super().__init__('rrt_planner_with_calibration')

        # --- Calibration ---
        self.vicon_topic = '/vrpn_mocap/iRobot_create3_te/pose'
        self.calib = ViconCalibrator(self.vicon_topic)
        rclpy.spin_once(self.calib, timeout_sec=0.1)
        T_v_from_r, self.theta = self.calib.run_interactive()
        self.T_r_from_v = np.linalg.inv(T_v_from_r)  # Vicon -> Robot(odom)

        # --- ROS wiring ---
        qos_gazebo = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(PoseStamped, '/gazebo_robot_pose',
                                 self._gazebo_pose_cb, qos_gazebo)

        self.nav_client = ActionClient(self, NavigateToPosition, '/navigate_to_position')

        # Path topics
        self.path_pub     = self.create_publisher(Path,   '/robot_traveled_path', 10)
        self.rrt_path_pub = self.create_publisher(Path,   '/rrt_planned_path',    10)

        # Visuals
        self.traveled_marker_pub = self.create_publisher(Marker,      '/traveled_marker', 10)
        self.planned_marker_pub  = self.create_publisher(Marker,      '/planned_marker',  10)
        self.obstacles_pub       = self.create_publisher(MarkerArray, '/obstacles_markers', 10)
        self.tb_markers_pub      = self.create_publisher(MarkerArray, '/tb_markers', 10)  # NEW

        # State
        self.robot_path = Path(); self.robot_path.header.frame_id = FRAME
        self.map_bounds = {'x': (-10, 10), 'y': (-10, 10)}
        self.goal_g = (3.0, 1.0)
        self.obstacles = load_obstacles()
        self.get_logger().info(f"Loaded {len(self.obstacles)} obstacles.")
        for i, o in enumerate(self.obstacles):
            self.get_logger().info(f"[{i}] {o['type']} at ({o['pose']['x']:.2f},{o['pose']['y']:.2f})")

        self.current_pose_ps: Optional[PoseStamped] = None
        self.path_g: List[Tuple[float, float]] = []
        self.path_index = 0
        self.started = False
        self.sending = False
        self.reached = False
        self._have_path = False

        # Soft-brake state
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._paused_until: float = 0.0
        self._goal_handle = None
        self._brake_timer = None

        # TB state: latest pose & filtered velocity per bot
        self.tb_names = list(TB_NAMES)
        self.tb_state: Dict[str, Dict[str, object]] = {
            ns: {'pose': None, 'vel': (0.0, 0.0)} for ns in self.tb_names
        }
        # Subscribe to each TB odom
        for idx, ns in enumerate(self.tb_names):
            topic = f'/{ns}/odom'
            self.create_subscription(
                Odometry, topic,
                lambda msg, _ns=ns: self._tb_odom_cb(_ns, msg), 10
            )
            self.get_logger().info(f"Subscribed TB odom: {topic}")

        # Timers
        self.create_timer(PRED_DT_S, self._replan_timer)     # replan / safety
        self.create_timer(0.10,      self._tb_marker_tick)   # publish TB markers @10Hz

        # One-time: publish static obstacles as RViz markers
        self._publish_obstacles_markers()

        self.get_logger().info("\nCalibration complete. Planner is ready.\n")

    # ---- Gazebo(odom) → Vicon → Robot(odom) ----
    def odom_to_robot(self, x_g: float, y_g: float) -> Tuple[float, float]:
        x_v, y_v = swap_scale_gazebo_to_vicon_xy(x_g, y_g)
        vec = np.array([[x_v], [y_v], [0.0], [1.0]])
        r = self.T_r_from_v @ vec
        return float(r[0]), float(r[1])

    # ----------------------------
    # Callbacks / timers
    # ----------------------------
    def _gazebo_pose_cb(self, msg: PoseStamped):
        self.current_pose_ps = msg

        # traveled path
        if self.started and not self.reached:
            p = PoseStamped()
            p.header.frame_id = FRAME
            p.header.stamp = self.get_clock().now().to_msg()
            p.pose = msg.pose
            self.robot_path.poses.append(p)
            self.path_pub.publish(self.robot_path)
            self.traveled_marker_pub.publish(
                self._path_to_marker(self.robot_path, COL_TRAVELED, "traveled", 0)
            )

        # initial plan
        if not self.started:
            sx, sy = msg.pose.position.x, msg.pose.position.y
            self.get_logger().info(f"Start (Gazebo): ({sx:.2f}, {sy:.2f}) → RRT planning…")
            self.path_g, _ = rrt((sx, sy), self.goal_g, self.map_bounds, self.obstacles)
            if not self.path_g:
                self.get_logger().error("RRT failed to find a path.")
                return
            self._publish_planned_path_and_marker()
            self.get_logger().info(f"RRT produced {len(self.path_g)} waypoints.")
            self._have_path = True
            self.started = True
            # be nice to action server the first time
            while not self.nav_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().info("Waiting for /navigate_to_position …")
            self._waypoint_tick()

    def _waypoint_tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now < self._paused_until:
            if self._brake_timer is None:
                self._soft_brake(self._paused_until - now)
            return
        if self.sending or self.reached:
            return
        if self.path_index >= len(self.path_g):
            self.get_logger().info('Final goal reached. 🎯')
            self.reached = True
            return

        xg, yg = self.path_g[self.path_index]
        xr, yr = self.odom_to_robot(xg, yg)

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

        goal = NavigateToPosition.Goal()
        goal.goal_pose.header.frame_id = FRAME
        goal.goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal.goal_pose.pose.position.x = xr
        goal.goal_pose.pose.position.y = yr
        goal.goal_pose.pose.orientation = quat_from_yaw(yaw)
        goal.achieve_goal_heading = False

        self.sending = True
        self.nav_client.send_goal_async(goal).add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        self._goal_handle = goal_handle  # save for cancel on brake
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected.')
            self.sending = False
            return
        self.get_logger().info('Goal accepted; waiting for result…')
        goal_handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result()
        self.sending = False
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Reached RRT waypoint ✅')
            self.path_index += 1
        else:
            self.get_logger().warn(f'NavigateToPosition result: status={result.status}')
            self.path_index += 1
        if self.path_index >= len(self.path_g):
            self.get_logger().info('Final goal reached. 🎯')
            self.reached = True
        else:
            self._waypoint_tick()

    # ----------------------------
    # TurtleBots (odom, markers, replan)
    # ----------------------------
    def _tb_odom_cb(self, ns: str, msg: Odometry):
        # Update pose & low-pass velocity per TB
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        prev_vx, prev_vy = self.tb_state[ns]['vel']
        self.tb_state[ns]['pose'] = (x, y)
        self.tb_state[ns]['vel']  = (0.7 * prev_vx + 0.3 * vx,
                                     0.7 * prev_vy + 0.3 * vy)

    def _tb_marker_tick(self):
        # Publish discs (and labels) for all TBs
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()

        # Clear old markers first (helps if nodes restart)
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        for i, ns in enumerate(self.tb_names):
            pose = self.tb_state[ns]['pose']
            if pose is None:
                continue
            x, y = pose
            r, g, b, a = TB_COLORS[i % len(TB_COLORS)]

            disc = Marker()
            disc.header.frame_id = FRAME
            disc.header.stamp = now
            disc.ns = f"{ns}_marker"
            disc.id = 1000 + i * 10
            disc.type = Marker.CYLINDER
            disc.action = Marker.ADD
            disc.pose.orientation.w = 1.0
            disc.pose.position.x = x
            disc.pose.position.y = y
            disc.pose.position.z = 0.0
            rad = TB_RADIUS + 0.02
            disc.scale.x = disc.scale.y = 2.0 * rad
            disc.scale.z = 0.02
            disc.color.r, disc.color.g, disc.color.b, disc.color.a = r, g, b, a
            arr.markers.append(disc)

            if TB_LABELS:
                label = Marker()
                label.header.frame_id = FRAME
                label.header.stamp = now
                label.ns = f"{ns}_label"
                label.id = 1000 + i * 10 + 1
                label.type = Marker.TEXT_VIEW_FACING
                label.action = Marker.ADD
                label.pose.orientation.w = 1.0
                label.pose.position.x = x
                label.pose.position.y = y
                label.pose.position.z = 0.18
                label.scale.z = 0.15
                label.color.r = label.color.g = label.color.b = 1.0
                label.color.a = 0.95
                label.text = ns
                arr.markers.append(label)

        if arr.markers:
            self.tb_markers_pub.publish(arr)

    def _replan_timer(self):
        # Only act when navigating and we have a path & a current pose
        if not self.started or self.reached or not self._have_path or self.current_pose_ps is None:
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        rx = self.current_pose_ps.pose.position.x
        ry = self.current_pose_ps.pose.position.y

        # Predict motion for each TB, and detect approach
        preds_by_ns: Dict[str, List[Tuple[float, float]]] = {}
        approaching_by_ns: Dict[str, bool] = {}

        for ns in self.tb_names:
            pose = self.tb_state[ns]['pose']
            vel  = self.tb_state[ns]['vel']
            if pose is None or vel is None:
                continue
            tbx, tby = pose
            tvx, tvy = vel

            # approach if velocity points roughly toward robot
            # (dot(v_tb, vector_to_robot) > 0)
            vx_to_r = (rx - tbx)
            vy_to_r = (ry - tby)
            approaching = (tvx * vx_to_r + tvy * vy_to_r) > 0.0
            approaching_by_ns[ns] = approaching

            # predictions
            pts = []
            t = 0.0
            while t <= PRED_HORIZON_S:
                pts.append((tbx + tvx * t, tby + tvy * t))
                t += PRED_DT_S
            preds_by_ns[ns] = pts

        # Emergency soft brake + (optional) replan
        # Also record if any bot within EMERGENCY_RADIUS (regardless of approach)
        emergency_hit = False
        for ns, pts in preds_by_ns.items():
            # check near-term predictions inside EMERGENCY_RADIUS
            if self.current_pose_ps is None:  # just in case
                continue
            steps = max(1, int(BRAKE_TTC_S / PRED_DT_S) if PRED_DT_S > 0 else 1)
            for px, py in pts[:steps]:
                if math.hypot(px - rx, py - ry) <= EMERGENCY_RADIUS:
                    emergency_hit = True
                    break
            if emergency_hit:
                break

        if emergency_hit:
            self._soft_brake(SOFT_BRAKE_S)

        # Debounce replans
        if now - getattr(self, '_last_replan_time', -1e9) < REPLAN_COOLDOWN_S:
            return

        # Replan criteria:
        # 1) Any approaching TB whose predicted centers intersect our corridor; OR
        # 2) Any TB inside emergency radius (we also brake above)
        replan_needed = False

        # Corridor check against approaching bots only
        for ns, pts in preds_by_ns.items():
            if not approaching_by_ns.get(ns, False):
                continue
            if self._preds_intersect_corridor(pts, self.path_g, CORRIDOR_W):
                replan_needed = True
                break

        # Also replan if an emergency hit is detected
        if emergency_hit:
            replan_needed = True

        if not replan_needed:
            # If pause window elapsed and we're idle, try to resume progression
            if now >= self._paused_until and not self.sending and not self.reached:
                self._waypoint_tick()
            return

        # Build dynamic discs from all TB predictions (subsampled)
        dyn_discs = []
        for ns, pts in preds_by_ns.items():
            for cx, cy in pts[::DYN_SUBSAMPLE]:
                dyn_discs.append({
                    "type": "sphere",
                    "pose": {"x": float(cx), "y": float(cy)},
                    "radius": float(TB_RADIUS + SAFETY_BUF)
                })
        obstacles = list(self.obstacles) + dyn_discs

        # Start from current pose (Gazebo/odom)
        sx = rx
        sy = ry
        new_path, _ = rrt((sx, sy), self.goal_g, self.map_bounds, obstacles)
        if new_path:
            self.path_g = new_path
            self.path_index = 0
            self._publish_planned_path_and_marker()
            self._have_path = True
            self._last_replan_time = now

        # If pause window elapsed and we're idle, try to resume progression
        if now >= self._paused_until and not self.sending and not self.reached:
            self._waypoint_tick()

    # ----------------------------
    # Helpers
    # ----------------------------
    def _soft_brake(self, seconds: float = SOFT_BRAKE_S):
        now = self.get_clock().now().nanoseconds * 1e-9
        self._paused_until = max(self._paused_until, now + max(0.0, seconds))
        try:
            if self._goal_handle is not None:
                self._goal_handle.cancel_goal_async()
        except Exception:
            pass
        if self._brake_timer is None:
            self._brake_timer = self.create_timer(0.1, self._brake_timer_cb)
        try:
            self.cmd_pub.publish(Twist())
        except Exception:
            pass

    def _brake_timer_cb(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now >= self._paused_until:
            if self._brake_timer is not None:
                self._brake_timer.cancel()
                self._brake_timer = None
            return
        try:
            self.cmd_pub.publish(Twist())
        except Exception:
            pass

    def _publish_planned_path_and_marker(self):
        # Publish thin Path
        rrt_msg = Path()
        rrt_msg.header.frame_id = FRAME
        now = self.get_clock().now().to_msg()
        for xg, yg in self.path_g:
            ps = PoseStamped()
            ps.header.frame_id = FRAME
            ps.header.stamp = now
            ps.pose.position.x = xg
            ps.pose.position.y = yg
            ps.pose.orientation.w = 1.0
            rrt_msg.poses.append(ps)
        self.rrt_path_pub.publish(rrt_msg)

        # Publish thick marker
        planned_m = self._path_to_marker(rrt_msg, COL_PLANNED, ns="planned", marker_id=0)
        self.planned_marker_pub.publish(planned_m)

    @staticmethod
    def _seg_point_dist(px, py, ax, ay, bx, by) -> float:
        abx, aby = bx - ax, by - ay
        apx, apy = px - ax, py - ay
        ab2 = abx*abx + aby*aby
        if ab2 <= 1e-12:
            return math.hypot(apx, apy)
        t = max(0.0, min(1.0, (apx*abx + apy*aby) / ab2))
        cx, cy = ax + t*abx, ay + t*aby
        return math.hypot(px - cx, py - cy)

    def _preds_intersect_corridor(self, pred_pts: List[Tuple[float,float]],
                                  path_xy: List[Tuple[float,float]],
                                  corridor_half_w: float) -> bool:
        if not path_xy or len(path_xy) < 2:
            return False
        for (px, py) in pred_pts:
            for (a, b) in zip(path_xy[:-1], path_xy[1:]):
                d = self._seg_point_dist(px, py, a[0], a[1], b[0], b[1])
                if d <= corridor_half_w:
                    return True
        return False

    # ---- Visualization helpers ----
    def _path_to_marker(self, path_msg: Path, color_rgba, ns: str, marker_id: int) -> Marker:
        r, g, b, a = color_rgba
        m = Marker()
        m.header.frame_id = path_msg.header.frame_id or FRAME
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = marker_id
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = LINE_WIDTH_M
        m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, a
        m.pose.orientation.w = 1.0
        m.lifetime.sec = 0
        m.points = [p.pose.position for p in path_msg.poses]
        return m

    def _publish_obstacles_markers(self):
        arr = MarkerArray()
        base_id = 100
        for i, obs in enumerate(self.obstacles):
            m = Marker()
            m.header.frame_id = FRAME
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "obstacles"
            m.id = base_id + i
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            r, g, b, a = COL_OBS
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, a

            m.pose.position.x = float(obs['pose']['x'])
            m.pose.position.y = float(obs['pose']['y'])
            m.pose.position.z = 0.0

            if obs['type'] in ('sphere', 'cylinder'):
                rad = float(obs['radius'])
                if obs['type'] == 'sphere':
                    m.type = Marker.SPHERE
                    m.scale.x = m.scale.y = m.scale.z = 2.0 * rad
                else:
                    m.type = Marker.CYLINDER
                    m.scale.x = m.scale.y = 2.0 * rad
                    m.scale.z = 0.01
            elif obs['type'] == 'box':
                m.type = Marker.CUBE
                sx, sy, *_ = obs['size']
                m.scale.x = float(sx)
                m.scale.y = float(sy)
                m.scale.z = 0.01
            else:
                continue
            arr.markers.append(m)

        clear = Marker(); clear.action = Marker.DELETEALL
        arr.markers.insert(0, clear)
        self.obstacles_pub.publish(arr)


# ----------------------------
# main
# ----------------------------
def main(args=None):
    rclpy.init(args=args)
    node = RRTPlannerWithCalibration()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        node.calib.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
