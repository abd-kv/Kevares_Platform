import rclpy
from rclpy.node import Node
from geopy.distance import geodesic
import pyproj
from std_msgs.msg import Int32MultiArray, Float64
from sensor_msgs.msg import NavSatFix
import math
import csv
import numpy as np

class NavigationAndControlNode(Node):
    def __init__(self):
        super().__init__('navigation_and_control')

        # Publisher
        self.publisher = self.create_publisher(Int32MultiArray, 'movement_commands', 10)

        # Subscribers
        self.create_subscription(NavSatFix, '/gps/gps_plugin/out', self.gps_callback, 10)
        self.create_subscription(Float64, '/gps_heading', self.heading_callback, 10)

        
        # Load lat/lon waypoints
        self.waypoints = self.load_waypoints('/home/user/abd_ws_2/src/tests/tests/zigzag_paths/final_combined_waypoints.csv')
        self.current_waypoint_index = 0

        # State
        self.current_lat = None
        self.current_lon = None
        self.heading = None  # degrees
        self.speed = 500

        # Projection transformer
        #self.transformer = None

        # Control parameter
        self.k = -1  # negative for right-hand drive coordinate frame

        # Timer
        self.create_timer(0.01, self.navigate)  # 5 Hz

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

        # Interpolation between waypoints (optional)
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

    # Added New Here

    def latlon_to_xy(self, lat1, lon1, lat2, lon2):
        """Convert lat/lon delta to meters (approximate)."""
        avg_lat_rad = math.radians((lat1 + lat2) / 2.0)
        dx = (lon2 - lon1) * 111000 * math.cos(avg_lat_rad)
        dy = (lat2 - lat1) * 111000
        return dx, dy


    def stanley_control(self, lat, lon, yaw_rad, v, lookahead_dist=2, search_window=3.0):
        if self.current_waypoint_index >= len(self.waypoints):
            return 0.0

        nearest_index = self.current_waypoint_index
        min_dist = float('inf')

        # Find nearest waypoint in window
        for i in range(self.current_waypoint_index, len(self.waypoints)):
            wp_lat, wp_lon = self.waypoints[i]
            dx, dy = self.latlon_to_xy(lat, lon, wp_lat, wp_lon)
            dist = math.hypot(dx, dy)
            if dist < min_dist and dist < search_window:
                min_dist = dist
                nearest_index = i
            elif dist > search_window:
                break

        # Lookahead index
        lookahead_index = nearest_index
        for j in range(nearest_index + 1, len(self.waypoints)):
            dx, dy = self.latlon_to_xy(*self.waypoints[nearest_index], *self.waypoints[j])
            dist = math.hypot(dx, dy)
            if dist >= lookahead_dist:
                lookahead_index = j
                break

        # Path yaw
        dx_path, dy_path = self.latlon_to_xy(*self.waypoints[nearest_index], *self.waypoints[lookahead_index])
        path_yaw = math.atan2(dy_path, dx_path)

        heading_error = math.atan2(math.sin(path_yaw - yaw_rad), math.cos(path_yaw - yaw_rad))

        # Cross-track error
        dx_ct, dy_ct = self.latlon_to_xy(lat, lon, *self.waypoints[nearest_index])
        cross_track_error = dx_ct * math.sin(path_yaw) - dy_ct * math.cos(path_yaw)

        steer_angle = heading_error + math.atan2(self.k * cross_track_error, v)
        steer_deg = max(-60, min(60, math.degrees(steer_angle)))

        self.get_logger().info(f"Cross-track error: {cross_track_error:.2f} m, Steering angle: {steer_deg:.2f}°")

        self.current_waypoint_index = nearest_index
        return steer_deg
    
    def navigate(self):
        if None in (self.current_lat, self.current_lon, self.heading):
            return

        # Normalize heading to [0, 360)
        heading_deg = -(self.heading-90) % 360.0
        yaw_rad = math.radians(heading_deg)

        if self.current_waypoint_index >= len(self.waypoints) - 2:
            self.get_logger().info("Navigation complete.")
            return
        v = self.speed/1000
        steering_angle = self.stanley_control(self.current_lat, self.current_lon, yaw_rad,v=v)
        self.control_robot(steering_angle)


    def control_robot(self, steering_angle):
        max_steering_units = 576
        max_steering_angle = 60.0  # degrees

        # Clamp and normalize to get steering units
        # steering_angle = max(-max_steering_angle, min(max_steering_angle, steering_angle))
        # steer_units = int((steering_angle / max_steering_angle) * max_steering_units)

        steering = int(max_steering_units*steering_angle/max_steering_angle)

        # speed = self.speed  # Fixed speed

        msg = Int32MultiArray()
        msg.data = [self.speed, steering]
        self.publisher.publish(msg)

        self.get_logger().info(f"Command: Speed {self.speed}, Steering {steering}")
        # self.get_logger().info("Navigation complete.")

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


