#!/usr/bin/env python3
# tb3_random_walker.py
# Random-walk + reactive avoidance for TurtleBot3 in Gazebo.
# Subscribes:  scan
# Publishes:   cmd_vel
# Works best when obstacles are STATIC in the world.

import math
import random
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def sector_min(ranges, start_idx, end_idx) -> float:
    """Finite min over a slice of the scan."""
    if not ranges:
        return float('inf')
    start_idx = max(0, start_idx)
    end_idx = min(len(ranges), end_idx)
    seg = [r for r in ranges[start_idx:end_idx] if math.isfinite(r)]
    return min(seg) if seg else float('inf')


class RandomWalker(Node):
    def __init__(self):
        super().__init__('tb3_random_walker')

        # -------- Parameters (override with ROS params if desired) --------
        self.declare_parameter('forward_speed',      0.18)   # m/s
        self.declare_parameter('turn_speed',         1.00)   # rad/s (base)
        self.declare_parameter('min_clear',          0.70)   # avoid if front < this
        self.declare_parameter('hard_stop',          0.25)   # immediate backup if front < this
        self.declare_parameter('avoid_time',         1.00)   # keep turning this long (s)
        self.declare_parameter('backup_time',        0.90)   # back up this long (s)
        self.declare_parameter('spin_time',          1.30)   # spin-in-place time when stuck (s)
        self.declare_parameter('stuck_window',       6.00)   # if constrained this long → spin (s)
        self.declare_parameter('front_width_deg',    70.0)   # width of frontal sector
        self.declare_parameter('rate_hz',            12.0)
        # random heading jitter while cruising
        self.declare_parameter('jitter_period_min',  3.0)
        self.declare_parameter('jitter_period_max',  6.0)
        self.declare_parameter('jitter_mag_rad',     0.35)

        p = self.get_parameter
        self.fwd        = float(p('forward_speed').value)
        self.omega      = float(p('turn_speed').value)
        self.min_clear  = float(p('min_clear').value)
        self.hard_stop  = float(p('hard_stop').value)
        self.avoid_dt   = Duration(seconds=float(p('avoid_time').value))
        self.backup_dt  = Duration(seconds=float(p('backup_time').value))
        self.spin_dt    = Duration(seconds=float(p('spin_time').value))
        self.stuck_win  = Duration(seconds=float(p('stuck_window').value))
        self.front_w    = math.radians(float(p('front_width_deg').value))
        self.rate_hz    = float(p('rate_hz').value)
        self.jit_min    = float(p('jitter_period_min').value)
        self.jit_max    = float(p('jitter_period_max').value)
        self.jit_mag    = float(p('jitter_mag_rad').value)

        # -------- State --------
        self._ranges: Optional[list[float]] = None
        self._angle_inc: Optional[float] = None
        self._angle_min: Optional[float] = None

        # timers / phases
        now = self.get_clock().now()
        self._avoid_until  = now
        self._backup_until = now
        self._spin_until   = now
        self._last_jitter_at = now
        self._next_jitter_dt = Duration(seconds=random.uniform(self.jit_min, self.jit_max))
        self._constrained_since: Optional[Duration] = None  # None means free

        self._avoid_dir = 1.0  # +1 left (CCW), -1 right (CW)

        # I/O
        self._sub_scan = self.create_subscription(LaserScan, 'scan', self._scan_cb, 10)
        self._pub_twist = self.create_publisher(Twist, 'cmd_vel', 10)
        self._timer = self.create_timer(1.0 / self.rate_hz, self._tick)

        self.get_logger().info("Random walker: bounce-back, avoid, unstick spin.")

    # -------------------- Callbacks --------------------
    def _scan_cb(self, msg: LaserScan):
        self._ranges = list(msg.ranges)
        self._angle_inc = msg.angle_increment
        self._angle_min = msg.angle_min

    # -------------------- Control Loop --------------------
    def _tick(self):
        tw = Twist()
        now = self.get_clock().now()

        # No scan yet → stay still
        if self._ranges is None or self._angle_inc is None or self._angle_min is None:
            self._pub_twist.publish(tw)
            return

        # Build sector indices around 0°
        total = len(self._ranges)
        center_idx = int((-self._angle_min) / self._angle_inc)  # index of 0 rad
        half = int((self.front_w / 2.0) / self._angle_inc)

        f_start = clamp(center_idx - half, 0, total)  # front sector
        f_end   = clamp(center_idx + half, 0, total)

        # Wider side sectors for “which way to turn”
        left_min  = sector_min(self._ranges, int(center_idx + half), int(center_idx + 3 * half))
        right_min = sector_min(self._ranges, int(center_idx - 3 * half), int(center_idx - half))
        front_min = sector_min(self._ranges, int(f_start), int(f_end))

        # 1) If currently spinning to unstick, keep spinning
        if now < self._spin_until:
            tw.angular.z = self.omega * random.choice([-1.0, 1.0])
            tw.linear.x = 0.0
            self._set_constrained_since(now)
            self._pub_twist.publish(tw)
            return

        # 2) If currently backing up, keep backing up
        if now < self._backup_until:
            tw.linear.x = -0.10
            tw.angular.z = 0.0
            self._set_constrained_since(now)
            self._pub_twist.publish(tw)
            return

        # 3) If way too close → start backup phase
        if front_min < self.hard_stop:
            self._backup_until = now + self.backup_dt
            # pick a random spin direction for after the backup
            self._avoid_dir = random.choice([-1.0, 1.0])
            self._set_constrained_since(now)
            tw.linear.x = -0.10
            tw.angular.z = 0.0
            self._pub_twist.publish(tw)
            return

        # 4) If in avoid phase, keep turning the chosen direction
        if now < self._avoid_until:
            tw.angular.z = self._avoid_dir * self.omega * 0.8
            tw.linear.x = 0.05
            self._set_constrained_since(now)
            self._pub_twist.publish(tw)
            return

        # 5) Start avoid phase if front too close; choose turn away from nearer side
        if front_min < self.min_clear:
            # if left has *more* clearance than right, turn left; else right
            self._avoid_dir = 1.0 if left_min > right_min else -1.0
            self._avoid_until = now + self.avoid_dt
            tw.angular.z = self._avoid_dir * self.omega * 0.8
            tw.linear.x = 0.03
            self._set_constrained_since(now)
            self._pub_twist.publish(tw)
            return

        # 6) Free to cruise: small forward, optional jitter
        tw.linear.x = self.fwd
        tw.angular.z = 0.0

        # Occasional random heading change to break symmetry
        if (now - self._last_jitter_at) > self._next_jitter_dt:
            tw.angular.z = random.choice([-1.0, 1.0]) * self.jit_mag
            self._last_jitter_at = now
            self._next_jitter_dt = Duration(seconds=random.uniform(self.jit_min, self.jit_max))

        # out of constrained behavior → clear timer
        self._constrained_since = None

        # 7) If we’ve been constrained too long overall, trigger an unstick spin
        if self._should_unstick(now):
            self._spin_until = now + self.spin_dt
            tw = Twist()  # will spin next tick
        self._pub_twist.publish(tw)

    # -------------------- Helpers --------------------
    def _set_constrained_since(self, now):
        if self._constrained_since is None:
            self._constrained_since = now

    def _should_unstick(self, now) -> bool:
        """If we stayed in avoid/backup for > stuck_window, trigger a spin."""
        if self._constrained_since is None:
            return False
        return (now - self._constrained_since) > self.stuck_win

    def destroy_node(self):
        # stop on shutdown
        try:
            self._pub_twist.publish(Twist())
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RandomWalker()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

