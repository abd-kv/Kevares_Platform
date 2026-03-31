import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, Int32MultiArray, Float64
import math
import csv
import pyproj

# Helper functions for calculations
def calculate_distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def calculate_angle(x1, y1, x2, y2):
    return math.atan2(y2 - y1, x2 - x1)

class NavigationAndControlNode(Node):
    def __init__(self):
        super().__init__('navigation_and_control')

        # Publisher
        self.publisher = self.create_publisher(Int32MultiArray, 'movement_commands', 10)

        # Subscribers
        #self.pose_sub = self.create_subscription(PoseStamped, '/calculated_pose', self.pose_callback, 10)
        # self.yaw_sub = self.create_subscription(Float32, '/yaw_degrees', self.yaw_callback, 10)
        self.create_subscription(NavSatFix, '/gps/gps_plugin/out', self.gps_callback, 10)
        
        #self.heading = self.create_subscription(Float32, '/gps_heading', self.heading_callback, 10)

        self.create_subscription(Float64, '/gps_heading', self.heading_callback, 10)
        self.heading = None  # Add this to store the actual float value

        # Waypoints from CSV file
        self.waypoints = self.load_waypoints('/home/user/abd_ws_2/src/tests/tests/zigzag_paths/combined_zigzag_path_waypoints.csv')
        self.current_waypoint_index = 0

        # Robot state
        self.current_x = None
        self.current_y = None
        self.yaw = None  # Stores the latest yaw value

        # GPS state
        self.current_lat = None
        self.current_lon = None

    def load_waypoints(self, filepath):
        waypoints = []
        try:
            with open(filepath, 'r') as file:
                reader = csv.reader(file)
                for row_number, row in enumerate(reader, start=1):
                    try:
                        lat = float(row[1])
                        lon = float(row[0])
                        waypoints.append((lat, lon))
                    except (ValueError, IndexError):
                        self.get_logger().warn(f"Skipping invalid row {row_number}: {row}")
        except FileNotFoundError:
            self.get_logger().error(f"Waypoint file '{filepath}' not found.")
            exit()
        return waypoints

    # def pose_callback(self, msg):
    #     self.current_x = msg.pose.position.x
    #     self.current_y = msg.pose.position.y

    def gps_callback(self, msg):
        self.current_lat = float(msg.latitude)
        self.current_lon = float(msg.longitude)
        self.get_logger().info(f"GPS Callback: Lat: {self.current_lat}, Lon: {self.current_lon}")

    # def yaw_callback(self, msg):
    #     self.yaw = msg.data
    #     self.get_logger().debug(f"Yaw Callback: {self.yaw:.2f} degrees")

    def heading_callback(self, msg):
        self.heading = msg.data
        self.get_logger().debug(f" Heading Callback: {self.heading:.2f} degrees")

    def navigate(self):
        # if self.current_x is None or self.current_y is None or self.yaw is None:
        #     self.get_logger().info("Waiting for valid pose and orientation data...")
        #     return

        if self.current_lon is None or self.current_lat is None:
            self.get_logger().info("Waiting for valid GPS data...")
            return
        
        if self.heading is None:
            self.get_logger().info("Waiting for valid HEADING data...")
            self.heading = 0.0
            return

        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info("All waypoints reached.")
            return

        target_lat, target_lon = self.waypoints[self.current_waypoint_index]
        self.get_logger().info(f"Current Target: Lat {target_lat}, Lon {target_lon}")

        # # Convert GPS waypoints to local X, Y coordinates
        # target_x, target_y = self.convert_gps_to_local(target_lat, target_lon)

        # Calculate distance and angle to the target
        distance = calculate_distance(-self.current_lon, -self.current_lat, -target_lon, -target_lat)
        alpha = calculate_angle( self.current_lat, self.current_lon, target_lat , target_lon )

        alpha = math.degrees(alpha) # NORTH is 0 degrees
        
        #alpha = (alpha + 450) % 360 # EAST is 0 degrees

        alpha = (alpha - self.heading)  # Use yaw instead of theta

        if alpha > 180:
            alpha -= 360
        elif alpha < -180:
            alpha += 360

        self.get_logger().info(f"Distance to Target: {distance*10000:.2f}, Angle to Target: {alpha:.2f} degrees, Heading: {self.heading}")

        if distance < 0.000015:  # Threshold for reaching the waypoint #< 0.000009
            self.get_logger().info(f"Reached waypoint {self.current_waypoint_index}")
            self.current_waypoint_index += 1
        else:
            self.control_robot(alpha)

    # def convert_gps_to_local(self, lat, lon):
    #     if self.current_x is None or self.current_y is None:
    #         self.get_logger().warn("Current position not initialized; cannot convert GPS to local coordinates.")
    #         return 0.0, 0.0

    #     if self.current_lat is None or self.current_lon is None:
    #         self.get_logger().warn("Base GPS position not initialized; cannot convert GPS to local coordinates.")
    #         return 0.0, 0.0

    #     base_lat = self.current_lat
    #     base_lon = self.current_lon 

    #     geod = pyproj.Geod(ellps="WGS84")
    #     _, _, distance_x = geod.inv(base_lon, base_lat, lon, base_lat)
    #     _, _, distance_y = geod.inv(base_lon, base_lat, base_lon, lat)

    #     if lon > base_lon:
    #         distance_x = -distance_x
    #     if lat < base_lat:
    #         distance_y = -distance_y

    #     self.get_logger().info(f"Converted GPS to Local: Lat {lat}, Lon {lon} -> x: {distance_x}, y: {distance_y}")
    #     return distance_x, distance_y

    def control_robot(self, alpha):
        max_steering = 576
        steering_interval = 1

        turn = 45 # 90
        clamped_alpha = max(-turn, min(alpha, turn))
        normalized_alpha = clamped_alpha / turn
        steering = round(normalized_alpha * max_steering / steering_interval) * steering_interval

        msg = Int32MultiArray()
        speed = 500

        speed = int(speed)
        steer = max(-576, min(576, int(steering)))  
        msg.data = [speed, -steer]

        self.get_logger().info(f"Publishing: Speed {speed:.2f}, Steering {steer:.2f}")
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationAndControlNode()

    def timer_callback():
        node.navigate()

    timer_period = 0.01  # 10 Hz
    node.create_timer(timer_period, timer_callback)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down node.")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
