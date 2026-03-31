#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float64, Float32, String
from sensor_msgs.msg import NavSatFix
import math
import csv
import numpy as np
import os

class PerimeterFollowNode(Node):
    """
    Exact perimeter follower (no zig-zag coverage):
    - Follows the recorded perimeter polyline segment-by-segment (i -> i+1)
    - Keeps the tracking target ON the current segment (small lookahead ahead on segment)
    - Switches to the next segment only when the endpoint is reached (within arrival radius)
    - Optional corner slow-down to help hardware make tight turns while preserving geometry

    Topics (compatible with your current setup):
      Subscriptions:
        /gps/gps_plugin/out   : sensor_msgs/NavSatFix    (latitude, longitude)
        /gps_heading          : std_msgs/Float64         (degrees, 0°=North, clockwise positive)
        /obstacle_speed_factor: std_msgs/Float32         (0.0..1.0)
        /stanley_start_cmd    : std_msgs/String          ("start"/"stop")
        /perimeter_name       : std_msgs/String          (folder name inside perimeter_dir)

      Publication:
        movement_commands     : std_msgs/Int32MultiArray [speed, steering_units]
                               (steering_units maps ±60° → ±576 just like your node)
    """

    def __init__(self):
        super().__init__('perimeter_follow_node')

        # === I/O ===
        self.pub_cmd = self.create_publisher(Int32MultiArray, 'movement_commands', 10)
        self.create_subscription(NavSatFix, '/gps/gps_plugin/out', self.gps_cb, 10)
        self.create_subscription(Float64, '/gps_heading', self.heading_cb, 10)
        self.create_subscription(Float32, '/obstacle_speed_factor', self.speed_factor_cb, 10)
        self.create_subscription(String, '/stanley_start_cmd', self.start_cb, 10)
        self.create_subscription(String, '/perimeter_name', self.perimeter_name_cb, 10)

        # === Files / waypoints ===
        self.perimeter_dir = "/home/user/abd_ws_2/src/tests/tests/perimeter"
        self.perimeter_name = None
        self.waypoints = []  # list of (lat, lon)

        # === Robot state ===
        self.lat = None
        self.lon = None
        self.heading_deg_north_cw = None  # 0=N, clockwise positive (as you publish)

        # === Output scaling ===
        self.base_speed = 500             # your previous speed units
        self.speed_factor = 1.0           # from obstacle node
        self.max_steer_units = 576
        self.max_steer_deg = 60.0

        # === Segment-follow parameters ===
        self.seg_index = 0                # active segment start index i (track i -> i+1)
        self.loop_perimeter = True        # wrap at end
        self.arrival_radius_seg = 0.8     # m to consider segment end reached
        self.target_ahead_m = 0.4         # m lookahead along segment (small => stick to line)
        self.wheelbase_m = 0.8            # approximate wheelbase for pure-pursuit geometry
        self.corner_slow_dist = 1.5       # start slowing this many meters before a sharp vertex
        self.corner_speed = 250           # speed cap near sharp turns
        self.turn_sharp_deg = 45.0        # threshold for "sharp" corner
        self.started_on_path = False      # whether we've snapped to the nearest segment

        # === Timer ===
        self.navigation_enabled = False
        self.create_timer(0.02, self.navigate)  # 50 Hz

    # --------------------
    # Subscriptions
    # --------------------
    def perimeter_name_cb(self, msg: String):
        self.perimeter_name = msg.data.strip()
        path = os.path.join(self.perimeter_dir, self.perimeter_name, 'final_combined_waypoints.csv')
        if not os.path.exists(path):
            self.get_logger().warn(f"Waypoint file not found: {path}")
            return
        self.waypoints = self.load_waypoints(path)
        if len(self.waypoints) < 2:
            self.get_logger().warn('Loaded waypoint list is too short.')
        else:
            self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints from {path}")
        # reset state so a new run snaps again
        self.started_on_path = False
        self.seg_index = 0

    def start_cb(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == 'start':
            self.started_on_path = False  # snap again on each start
            self.seg_index = 0
            self.navigation_enabled = True
            self.get_logger().info('Perimeter exact-follow: START')
        elif cmd == 'stop':
            self.navigation_enabled = False
            self.get_logger().info('Perimeter exact-follow: STOP')

    def speed_factor_cb(self, msg: Float32):
        self.speed_factor = max(0.0, min(1.0, float(msg.data)))
        self.get_logger().debug(f"speed_factor: {self.speed_factor:.2f}")

    def gps_cb(self, msg: NavSatFix):
        self.lat = float(msg.latitude)
        self.lon = float(msg.longitude)

    def heading_cb(self, msg: Float64):
        self.heading_deg_north_cw = float(msg.data)

    # --------------------
    # Waypoint loading
    # --------------------
    def load_waypoints(self, filepath, samples_per_segment: int = 10):
        """Load lon,lat CSV and lightly densify for smooth steering while preserving geometry."""
        raw = []
        with open(filepath, 'r') as f:
            for row in csv.reader(f):
                try:
                    lon = float(row[0]); lat = float(row[1])
                    raw.append((lat, lon))
                except Exception:
                    continue
        if len(raw) < 2:
            return raw

        dense = []
        for i in range(len(raw) - 1):
            lat0, lon0 = raw[i]
            lat1, lon1 = raw[i + 1]
            # keep endpoint exclusive to avoid duplicating; we'll append the true last point after loop
            for t in np.linspace(0.0, 1.0, samples_per_segment, endpoint=False):
                dense.append(((1 - t) * lat0 + t * lat1, (1 - t) * lon0 + t * lon1))
        dense.append(raw[-1])
        return dense

    # --------------------
    # Geometry helpers
    # --------------------
    @staticmethod
    def latlon_lerp(a_lat, a_lon, b_lat, b_lon, t: float):
        return (a_lat + t * (b_lat - a_lat), a_lon + t * (b_lon - a_lon))

    @staticmethod
    def wrap_heading_to_yaw_rad(heading_deg_north_cw: float) -> float:
        """Convert 0°=North (CW +) to ENU yaw: 0°=East (CCW +)."""
        heading_deg = (-(heading_deg_north_cw - 90.0)) % 360.0
        return math.radians(heading_deg)

    @staticmethod
    def xy_from_latlon(lat: float, lon: float, ref_lat: float, ref_lon: float):
        avg_lat = math.radians((lat + ref_lat) / 2.0)
        dx = (lon - ref_lon) * 111000.0 * math.cos(avg_lat)
        dy = (lat - ref_lat) * 111000.0
        return dx, dy

    def path_distance(self, a_lat, a_lon, b_lat, b_lon):
        dx, dy = self.xy_from_latlon(a_lat, a_lon, b_lat, b_lon)
        return math.hypot(dx, dy)

    def project_onto_segment(self, p_lat, p_lon, a_lat, a_lon, b_lat, b_lon):
        """Return (t_clamped, seg_len_m, proj_lat, proj_lon) with t in [0,1] along A->B."""
        ax, ay = 0.0, 0.0
        bx, by = self.xy_from_latlon(b_lat, b_lon, a_lat, a_lon)
        px, py = self.xy_from_latlon(p_lat, p_lon, a_lat, a_lon)
        vx, vy = bx - ax, by - ay
        v2 = vx * vx + vy * vy
        if v2 < 1e-6:
            return 0.0, 0.0, a_lat, a_lon
        t = ((px - ax) * vx + (py - ay) * vy) / v2
        t = max(0.0, min(1.0, t))
        seg_len = math.sqrt(v2)
        proj_lat, proj_lon = self.latlon_lerp(a_lat, a_lon, b_lat, b_lon, t)
        return t, seg_len, proj_lat, proj_lon

    def angle_between_segments_deg(self, a_lat, a_lon, b_lat, b_lon, c_lat, c_lon):
        """Unsigned angle (0..180) between AB and BC at B."""
        bx, by = 0.0, 0.0
        ax, ay = self.xy_from_latlon(a_lat, a_lon, b_lat, b_lon)
        cx, cy = self.xy_from_latlon(c_lat, c_lon, b_lat, b_lon)
        v1x, v1y = ax - bx, ay - by
        v2x, v2y = cx - bx, cy - by
        n1 = math.hypot(v1x, v1y); n2 = math.hypot(v2x, v2y)
        if n1 < 1e-6 or n2 < 1e-6:
            return 0.0
        cosang = max(-1.0, min(1.0, (v1x * v2x + v1y * v2y) / (n1 * n2)))
        return math.degrees(math.acos(cosang))

    # --------------------
    # Start logic helpers
    # --------------------
    def snap_to_nearest_segment_once(self):
        if self.started_on_path or len(self.waypoints) < 2 or self.lat is None:
            return
        best_i, best_d = 0, float('inf')
        for i in range(len(self.waypoints) - 1):
            alat, alon = self.waypoints[i]
            blat, blon = self.waypoints[i + 1]
            t, seg_len, proj_lat, proj_lon = self.project_onto_segment(self.lat, self.lon, alat, alon, blat, blon)
            dx, dy = self.xy_from_latlon(self.lat, self.lon, proj_lat, proj_lon)
            d = math.hypot(dx, dy)
            if d < best_d:
                best_d = d
                best_i = i
        self.seg_index = best_i
        self.started_on_path = True
        self.get_logger().info(f"Snapped to segment {best_i}->{best_i+1} (dist {best_d:.2f} m)")

    # --------------------
    # Segment-follow steering & speed
    # --------------------
    def segment_follow_steer_and_speed(self, yaw_rad):
        n = len(self.waypoints)
        if n < 2:
            return 0.0, int(self.base_speed * self.speed_factor)

        i = self.seg_index
        alat, alon = self.waypoints[i]
        blat, blon = self.waypoints[(i + 1) % n]

        # Project current position onto active segment
        t, seg_len, proj_lat, proj_lon = self.project_onto_segment(self.lat, self.lon, alat, alon, blat, blon)
        dist_to_end = max(0.0, seg_len * (1.0 - t))

        # If close to segment end, advance to next segment
        if dist_to_end <= self.arrival_radius_seg:
            self.seg_index = (i + 1) % n if self.loop_perimeter else min(i + 1, n - 2)
            # Recompute with new segment
            i = self.seg_index
            alat, alon = self.waypoints[i]
            blat, blon = self.waypoints[(i + 1) % n]
            t, seg_len, proj_lat, proj_lon = self.project_onto_segment(self.lat, self.lon, alat, alon, blat, blon)
            dist_to_end = max(0.0, seg_len * (1.0 - t))

        # Choose target point ahead ALONG the segment (clamped to end)
        ahead = min(self.target_ahead_m, dist_to_end)
        t_target = 0.0 if seg_len < 1e-6 else min(1.0, t + ahead / max(seg_len, 1e-6))
        tlat, tlon = self.latlon_lerp(alat, alon, blat, blon, t_target)

        # Transform target into robot frame (x forward, y left)
        dx, dy = self.xy_from_latlon(self.lat, self.lon, tlat, tlon)
        x_fwd =  math.cos(yaw_rad) * dx + math.sin(yaw_rad) * dy
        y_left = -math.sin(yaw_rad) * dx + math.cos(yaw_rad) * dy
        Ld = max(0.1, math.hypot(x_fwd, y_left))

        # Pure-pursuit curvature and steering
        kappa = 2.0 * y_left / (Ld * Ld)
        steer_rad = math.atan(kappa * self.wheelbase_m)
        steer_deg = max(-self.max_steer_deg, min(self.max_steer_deg, math.degrees(steer_rad)))

        # Speed: optionally slow down near sharp corners to make the turn
        speed_cmd = int(self.base_speed * self.speed_factor)
        if n >= 3:
            i_prev = (i - 1) % n
            i_next = (i + 1) % n
            a_lat, a_lon = self.waypoints[i_prev]
            b_lat, b_lon = self.waypoints[i]
            c_lat, c_lon = self.waypoints[i_next]
            turn_deg = self.angle_between_segments_deg(a_lat, a_lon, b_lat, b_lon, c_lat, c_lon)
            if turn_deg <= self.turn_sharp_deg and dist_to_end <= self.corner_slow_dist:
                speed_cmd = min(speed_cmd, self.corner_speed)

        return steer_deg, speed_cmd

    # --------------------
    # Main loop
    # --------------------
    def navigate(self):
        if not self.navigation_enabled:
            return
        if None in (self.lat, self.lon, self.heading_deg_north_cw):
            return
        if len(self.waypoints) < 2:
            self.get_logger().warn('No valid waypoints loaded.')
            return

        # Convert heading to yaw
        yaw_rad = self.wrap_heading_to_yaw_rad(self.heading_deg_north_cw)

        # Snap once to the correct segment
        self.snap_to_nearest_segment_once()

        # Get steering and speed that keep us on the current segment
        steer_deg, speed_cmd = self.segment_follow_steer_and_speed(yaw_rad)

        # Publish command
        steering_units = int(self.max_steer_units * (steer_deg / self.max_steer_deg))
        msg = Int32MultiArray()
        msg.data = [int(speed_cmd), int(steering_units)]
        self.pub_cmd.publish(msg)
        self.get_logger().info(f"[Seg {self.seg_index}] Cmd: Speed {speed_cmd}, Steering {steering_units} (deg {steer_deg:.1f})")


def main(args=None):
    rclpy.init(args=args)
    node = PerimeterFollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
