
#!/usr/bin/env python3
# rrt_planner_with_auto_calibration_viz.py

import math
from typing import Tuple, List

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from action_msgs.msg import GoalStatus
from transforms3d.euler import euler2quat

from irobot_create_msgs.action import NavigateToPosition
from rrt_planner.rrt_planner import rrt, load_obstacles

# ----------------------------
# Tunables
# ----------------------------
SCALE = 2.25                     # Gazebo <-> Vicon scale factor you found
LINE_WIDTH_M = 0.06              # path line thickness (meters)
COL_TRAVELED = (1.0, 0.2, 0.2, 1.0)  # RGBA (red-ish)
COL_PLANNED  = (0.2, 1.0, 0.2, 1.0)  # RGBA (Green)
COL_OBS      = (0.2, 0.8, 1.0, 0.8)  # RGBA for obstacles (teal/blue)
FRAME = "odom"                    # everything shown in RViz in odom


# ----------------------------
# Helpers
# ----------------------------
def swap_scale_gazebo_to_vicon_xy(x_g: float, y_g: float) -> Tuple[float, float]:
    """
    Your fixed Gazebo->Vicon mapping:
        y_v =  x_g / SCALE
        x_v = -y_g / SCALE
    """
    y_v =  x_g / SCALE
    x_v = -y_g / SCALE
    return x_v, y_v


def quat_from_yaw(yaw: float) -> Quaternion:
    w, x, y, z = euler2quat(0.0, 0.0, yaw)
    q = Quaternion()
    q.w, q.x, q.y, q.z = w, x, y, z
    return q


# ----------------------------
# Calibration (two clicks)
# ----------------------------
class ViconCalibrator(Node):
    """Two-click calibration: capture (x1,y1), then move forward to get heading θ."""
    def __init__(self, vicon_topic: str):
        super().__init__('vicon_calibrator')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.sub = self.create_subscription(PoseStamped, vicon_topic, self._cb, qos)
        self.latest: PoseStamped | None = None

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


# ----------------------------
# Main Planner Node
# ----------------------------
class RRTPlannerWithCalibration(Node):
    def __init__(self):
        super().__init__('rrt_planner_with_calibration')

        # --- 1) CALIBRATION (blocking) ---
        self.vicon_topic = '/vrpn_mocap/iRobot_create3_te/pose'
        self.calib = ViconCalibrator(self.vicon_topic)
        T_v_from_r, self.theta = self.calib.run_interactive()
        # We need Vicon->Robot (robot/odom goals): take inverse
        self.T_r_from_v = np.linalg.inv(T_v_from_r)

        # --- 2) ROS wiring ---
        qos_gazebo = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.create_subscription(PoseStamped, '/gazebo_robot_pose',
                                 self._gazebo_pose_cb, qos_gazebo)

        self.nav_client = ActionClient(self, NavigateToPosition, '/navigate_to_position')

        # Path topics (thin lines, as before)
        self.path_pub     = self.create_publisher(Path,   '/robot_traveled_path', 10)
        self.rrt_path_pub = self.create_publisher(Path,   '/rrt_planned_path',    10)

        # New: thick RViz markers (easy to see on the floor)
        self.traveled_marker_pub = self.create_publisher(Marker,      '/traveled_marker', 10)
        self.planned_marker_pub  = self.create_publisher(Marker,      '/planned_marker',  10)
        self.obstacles_pub       = self.create_publisher(MarkerArray, '/obstacles_markers', 10)

        # State
        self.robot_path = Path()
        self.robot_path.header.frame_id = FRAME

        self.map_bounds = {'x': (-10, 10), 'y': (-10, 10)}
        self.goal_g = (3.0, 1.0)  # goal in Gazebo/odom
        self.obstacles = load_obstacles()
        self.get_logger().info(f"Loaded {len(self.obstacles)} obstacles.")
        for i,o in enumerate(self.obstacles):
            self.get_logger().info(f"[{i}] {o['type']} at ({o['pose']['x']:.2f},{o['pose']['y']:.2f})")


        self.current_pose_ps: PoseStamped | None = None
        self.path_g: List[Tuple[float, float]] = []
        self.path_index = 0
        self.started = False
        self.sending = False
        self.reached = False

        # Publish obstacle markers once
        self._publish_obstacles_markers()

        print("\nCalibration complete. Planner is ready.\n")

    # ---- Gazebo(odom) → Vicon → Robot(odom) ----
    def odom_to_robot(self, x_g: float, y_g: float) -> Tuple[float, float]:
        # Step 1: fixed scale+axis swap to Vicon
        x_v, y_v = swap_scale_gazebo_to_vicon_xy(x_g, y_g)
        # Step 2: calibrated Vicon -> Robot/odom
        vec = np.array([[x_v], [y_v], [0.0], [1.0]])
        r = self.T_r_from_v @ vec
        return float(r[0]), float(r[1])

    # ----------------------------
    # Callbacks / timers
    # ----------------------------
    def _gazebo_pose_cb(self, msg: PoseStamped):
        self.current_pose_ps = msg

        # Record traveled path (thin Path + thick Marker)
        if self.started and not self.reached:
            p = PoseStamped()
            p.header.frame_id = FRAME
            p.header.stamp = self.get_clock().now().to_msg()
            p.pose = msg.pose
            self.robot_path.poses.append(p)
            self.path_pub.publish(self.robot_path)

            trav_m = self._path_to_marker(self.robot_path, COL_TRAVELED, ns="traveled", marker_id=0)
            self.traveled_marker_pub.publish(trav_m)

        # On first pose: plan in Gazebo/odom
        if not self.started:
            sx, sy = msg.pose.position.x, msg.pose.position.y
            self.get_logger().info(f"Start (Gazebo): ({sx:.2f}, {sy:.2f}) → RRT planning…")

            self.path_g, _ = rrt((sx, sy), self.goal_g, self.map_bounds, self.obstacles)
            if not self.path_g:
                self.get_logger().error("RRT failed to find a path.")
                return

            # Publish planned path (thin) and thick marker
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

            planned_m = self._path_to_marker(rrt_msg, COL_PLANNED, ns="planned", marker_id=0)
            self.planned_marker_pub.publish(planned_m)

            self.get_logger().info(f"RRT produced {len(self.path_g)} waypoints.")
            self.started = True
            self._waypoint_tick()

    def _waypoint_tick(self):
        while not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Waiting for /navigate_to_position …")

        if (not self.started or self.reached or self.sending or
            self.path_index >= len(self.path_g)):
            return

        # next Gazebo waypoint → robot/odom
        xg, yg = self.path_g[self.path_index]
        xr, yr = self.odom_to_robot(xg, yg)
        self.get_logger().info(
            f"[{self.path_index+1}/{len(self.path_g)}] Gazebo→Robot: "
            f"({xg:.2f},{yg:.2f}) → ({xr:.2f},{yr:.2f})"
        )

        # skip if close
        if self.current_pose_ps:
            dx = xr - self.current_pose_ps.pose.position.x
            dy = yr - self.current_pose_ps.pose.position.y
            if math.hypot(dx, dy) < 0.10:
                self.path_index += 1
                self._waypoint_tick()
                return

        # heading using next transformed waypoint
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
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected.')
            self.sending = False
            return
        self.get_logger().info('Goal accepted—waiting result…')
        goal_handle.get_result_async().add_done_callback(self._result_cb)

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

    # ----------------------------
    # RViz Marker builders
    # ----------------------------
    def _path_to_marker(self, path_msg: Path, color_rgba, ns: str, marker_id: int) -> Marker:
        r, g, b, a = color_rgba
        m = Marker()
        m.header.frame_id = path_msg.header.frame_id or FRAME
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = marker_id
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = LINE_WIDTH_M                # thickness (meters)
        m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, a
        m.pose.orientation.w = 1.0
        m.lifetime.sec = 0                      # persist
        m.points = [p.pose.position for p in path_msg.poses]
        return m

    def _publish_obstacles_markers(self):
        """
        Visualize your rrt_planner.data/obstacles.json as a MarkerArray in 'odom'.
        Supported types: 'sphere', 'cylinder', 'box' (fields match your loader).
        """
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()

        for i, obs in enumerate(self.obstacles):
            m = Marker()
            m.header.frame_id = FRAME
            m.header.stamp = now
            m.ns = "obstacles"
            m.id = i
            m.action = Marker.ADD
            m.color.r, m.color.g, m.color.b, m.color.a = COL_OBS
            m.pose.orientation.w = 1.0
            m.lifetime.sec = 0

            ox = obs['pose']['x']
            oy = obs['pose']['y']
            m.pose.position.x = ox
            m.pose.position.y = oy
            m.pose.position.z = 0.0

            if obs['type'] in ('sphere', 'cylinder'):
                r = float(obs['radius'])
                if obs['type'] == 'sphere':
                    m.type = Marker.SPHERE
                    m.scale.x = m.scale.y = m.scale.z = 2.0 * r
                else:
                    m.type = Marker.CYLINDER
                    m.scale.x = m.scale.y = 2.0 * r
                    m.scale.z = 0.01  # flat on floor
            elif obs['type'] == 'box':
                m.type = Marker.CUBE
                sx, sy, *rest = obs['size']
                m.scale.x = float(sx)
                m.scale.y = float(sy)
                m.scale.z = 0.01
            else:
                # unknown type → skip
                continue

            arr.markers.append(m)

        # Also send a DELETEALL to clear old ones before adding (helps on relaunch)
        clear = Marker()
        clear.action = Marker.DELETEALL
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
