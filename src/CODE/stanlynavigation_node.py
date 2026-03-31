import rclpy
from rclpy.node import Node
from geopy.distance import geodesic
import pyproj
from std_msgs.msg import Int32MultiArray, Float64, Float32, String
from sensor_msgs.msg import NavSatFix
import math
import csv
import numpy as np
import os

class NavigationAndControlNode(Node):
    def __init__(self):
        super().__init__('navigation_and_control')

        # === ROS Pub/Sub ===
        self.publisher = self.create_publisher(Int32MultiArray, 'movement_commands', 10)
        self.create_subscription(NavSatFix, '/gps/gps_plugin/out', self.gps_callback, 10)
        self.create_subscription(Float64, '/gps_heading', self.heading_callback, 10)
        self.create_subscription(Float32, '/obstacle_speed_factor', self.speed_factor_callback, 10)
        self.create_subscription(String, '/stanley_start_cmd', self.start_callback, 10)
        self.create_subscription(String, '/perimeter_name', self.perimeter_name_callback, 10)

        # === Waypoints and Params ===
        self.perimeter_dir = "/home/user/abd_ws_2/src/tests/tests/perimeter"
        self.perimeter_name = None
        self.waypoints = []
        self.current_waypoint_index = 0
        self.current_lat = None
        self.current_lon = None
        self.heading = None
        self.speed = 500
        self.speed_factor = 1.0
        self.navigation_enabled = False
        self.k = -1
        self.stop_navigation = 1

        # === Timer ===
        self.create_timer(0.01, self.navigate)

    def perimeter_name_callback(self, msg: String):
        self.perimeter_name = msg.data.strip()
        path = os.path.join(self.perimeter_dir, self.perimeter_name, "final_combined_waypoints.csv")
        if os.path.exists(path):
            self.waypoints = self.load_waypoints(path)
            self.current_waypoint_index = 0
            self.get_logger().info(f"Loaded waypoints from: {path}")
        else:
            self.get_logger().warn(f" Waypoint file not found: {path}")

    def start_callback(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == "start":
            self.navigation_enabled = True
            self.get_logger().info("Navigation started via /stanley_start_cmd")
        elif cmd == "stop":
            self.navigation_enabled = False
            self.get_logger().info("Navigation stopped via /stanley_start_cmd")

    def speed_factor_callback(self, msg: Float32):
        self.speed_factor = max(0.0, min(1.0, msg.data))
        self.get_logger().info(f"Received speed factor: {self.speed_factor:.2f}")

    def load_waypoints(self, filepath):
        latlon = []
        with open(filepath, 'r') as file:
            reader = csv.reader(file)
            for row in reader:
                try:
                    lat = float(row[1])
                    lon = float(row[0])
                    latlon.append((lat, lon))
                except:
                    continue

        interpolated = []
        for i in range(len(latlon) - 1):
            lat0, lon0 = latlon[i]
            lat1, lon1 = latlon[i + 1]
            for t in np.linspace(0, 1, 101):
                lati = (1 - t) * lat0 + t * lat1
                loni = (1 - t) * lon0 + t * lon1
                interpolated.append((lati, loni))
        return interpolated

    def gps_callback(self, msg):
        self.current_lat = float(msg.latitude)
        self.current_lon = float(msg.longitude)

    def heading_callback(self, msg):
        self.heading = msg.data

    def latlon_to_xy(self, lat1, lon1, lat2, lon2):
        avg_lat_rad = math.radians((lat1 + lat2) / 2.0)
        dx = (lon2 - lon1) * 111000 * math.cos(avg_lat_rad)
        dy = (lat2 - lat1) * 111000
        return dx, dy

    def stanley_control(self, lat, lon, yaw_rad, v, lookahead_dist=2, search_window=3.0):
        if self.current_waypoint_index >= len(self.waypoints):
            self.stop_navigation = 0
            return 0.0
            
            
        self.stop_navigation = 1
        nearest_index = self.current_waypoint_index
        min_dist = float('inf')

        for i in range(self.current_waypoint_index, len(self.waypoints)):
            wp_lat, wp_lon = self.waypoints[i]
            dx, dy = self.latlon_to_xy(lat, lon, wp_lat, wp_lon)
            dist = math.hypot(dx, dy)
            if dist < min_dist and dist < search_window:
                min_dist = dist
                nearest_index = i
            elif dist > search_window:
                break
        if nearest_index == self.current_waypoint_index:
            min_dist = float('inf')
            # last_dist = float('inf')
            for i in range(self.current_waypoint_index, len(self.waypoints)):
                wp_lat, wp_lon = self.waypoints[i]
                dx, dy = self.latlon_to_xy(lat, lon, wp_lat, wp_lon)
                dist = math.hypot(dx, dy)

                if dist < min_dist:
                    min_dist = dist
                    nearest_index = i

                if dist > min_dist:
                    break  # passed closest point
        print(f'Actual distance = {dist}')
        lookahead_index = nearest_index
        for j in range(nearest_index + 1, len(self.waypoints)):
            dx, dy = self.latlon_to_xy(*self.waypoints[nearest_index], *self.waypoints[j])
            dist = math.hypot(dx, dy)
            if dist >= lookahead_dist:
                lookahead_index = j
                break

        dx_path, dy_path = self.latlon_to_xy(*self.waypoints[nearest_index], *self.waypoints[lookahead_index])
        path_yaw = math.atan2(dy_path, dx_path)
        heading_error = math.atan2(math.sin(path_yaw - yaw_rad), math.cos(path_yaw - yaw_rad))

        dx_ct, dy_ct = self.latlon_to_xy(lat, lon, *self.waypoints[nearest_index])
        cross_track_error = dx_ct * math.sin(path_yaw) - dy_ct * math.cos(path_yaw)

        steer_angle = heading_error + math.atan2(self.k * cross_track_error, v)
        steer_deg = max(-60, min(60, math.degrees(steer_angle)))

        self.get_logger().info(f"Cross-track error: {cross_track_error:.2f} m, Steering angle: {steer_deg:.2f}°")

        self.current_waypoint_index = nearest_index
        return steer_deg

    def navigate(self):
        if not self.navigation_enabled:
            return

        if None in (self.current_lat, self.current_lon, self.heading):
            return

        if len(self.waypoints) < 2:
            self.get_logger().warn("No valid waypoints loaded.")
            return

        heading_deg = -(self.heading - 90) % 360.0
        yaw_rad = math.radians(heading_deg)

        if self.current_waypoint_index >= len(self.waypoints) - 2:
            self.get_logger().info("Navigation complete.")
            return

        v = self.speed / 1000
        steering_angle = self.stanley_control(self.current_lat, self.current_lon, yaw_rad, v=v)
        self.control_robot(steering_angle)

    def control_robot(self, steering_angle):
        max_steering_units = 576
        max_steering_angle = 60.0

        steering = int(max_steering_units * steering_angle / max_steering_angle)

        speed = int(self.speed * self.speed_factor)
        msg = Int32MultiArray()
        msg.data = [speed, steering]
        self.publisher.publish(msg)
        # self.get_logger().info(f"actual command: Speed {self.speed}, Steering {steering}")
        self.get_logger().info(f"Command: Speed {self.speed}, Steering {steering}")


def main(args=None):
    rclpy.init(args=args)
    node = NavigationAndControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
