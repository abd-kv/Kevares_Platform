import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
import csv
from shapely.geometry import Point, Polygon

class PerimeterCheckerNode(Node):
    def __init__(self):
        super().__init__('perimeter_checker_node')

        # Path to perimeters
        self.perimeter_dir = "/home/user/abd_ws_2/src/tests/tests/perimeter"
        self.perimeters = self.load_perimeters_from_csv()

        self.subscription = self.create_subscription(
            NavSatFix,
            '/gps/gps_plugin/out',
            self.gps_callback,
            10
        )
        self.proximity_pub = self.create_publisher(String, '/perimeter_proximity_alert', 10)
        self.perimeter_name_pub = self.create_publisher(String, '/perimeter_name', 10)

    def load_perimeters_from_csv(self):
        perimeters = {}
        for folder in os.listdir(self.perimeter_dir):
            csv_path = os.path.join(self.perimeter_dir, folder, 'gps_data.csv')
            if os.path.isfile(csv_path):
                points = []
                with open(csv_path, 'r') as csvfile:
                    reader = csv.reader(csvfile)
                    for row in reader:
                        try:
                            lat = float(row[0])
                            lon = float(row[1])
                            points.append((lon, lat))  # (longitude, latitude) order for shapely
                        except (ValueError, IndexError):
                            continue  # Skip invalid rows
                if len(points) >= 3:  # Need at least 3 points to form a polygon
                    perimeters[folder] = Polygon(points)
        return perimeters

    def gps_callback(self, msg):
        current_pos = Point(msg.longitude, msg.latitude)
        closest_perimeter = None
        min_distance = float('inf')

        for name, polygon in self.perimeters.items():
            distance = polygon.distance(current_pos)
            if distance < min_distance:
                min_distance = distance
                closest_perimeter = name

        if closest_perimeter:
            # Publish friendly alert for Discord bot
            alert_msg = String()
            alert_msg.data = f"📍 Closest zone: {closest_perimeter}. Should I start moving?"
            self.proximity_pub.publish(alert_msg)
            self.get_logger().info(alert_msg.data)

            # Auto-publish detected perimeter name to /perimeter_name
            name_msg = String()
            name_msg.data = closest_perimeter
            self.perimeter_name_pub.publish(name_msg)
            self.get_logger().info(f"📤 Auto-published perimeter name: {closest_perimeter}")


def main(args=None):
    rclpy.init(args=args)
    node = PerimeterCheckerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
