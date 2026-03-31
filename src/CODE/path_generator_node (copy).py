#!/usr/bin/env python3

import csv
from std_msgs.msg import String
import pandas as pd
import matplotlib.pyplot as plt
from shapely import Point
from shapely.geometry import MultiPoint, LineString, Polygon, MultiPolygon
from shapely.ops import unary_union
from scipy.interpolate import CubicSpline, make_interp_spline
import numpy as np
import rclpy
from rclpy.node import Node

# -------------------------------
# Border Offset Utility Function
# -------------------------------
def generate_border_paths(boundary, offset_distance=0.00001, num_levels=2):
    border_paths = []
    current_boundary = boundary
    for level in range(num_levels + 1):
        border_path = list(current_boundary.exterior.coords)
        border_paths.append(border_path)
        new_boundary = current_boundary.buffer(-offset_distance)
        if new_boundary.is_empty:
            break
        if new_boundary.geom_type == 'MultiPolygon':
            new_boundary = max(new_boundary.geoms, key=lambda p: p.area)
        current_boundary = new_boundary
    return border_paths, current_boundary

class PathGenerator(Node):
    def __init__(self):
        super().__init__('path_generator_node')
        self.get_logger().info("Path Generator Node Initialized")

        self.perimeter_name = None
        self.file_path = None
        self.output_image = "primary_secondary_zigzag_path.png"

        self.row_spacing = 0.00002
        self.row_spacing_border = 0.00001
        self.point_density = 15

        self.name_sub = self.create_subscription(
            String,
            '/perimeter_name',
            self.perimeter_name_callback,
            10
        )
        self.trigger_sub = self.create_subscription(
            String,
            '/generate_path_trigger',
            self.trigger_callback,
            10
        )

    def perimeter_name_callback(self, msg):
        self.perimeter_name = msg.data.strip()
        self.file_path = f"/home/user/abd_ws_2/src/tests/tests/perimeter/{self.perimeter_name}/gps_data.csv"
        self.get_logger().info(f"Received perimeter name: {self.perimeter_name}")

    def trigger_callback(self, msg):
        if msg.data.strip().lower() == "start":
            self.get_logger().info("🚀 Path generation trigger received from Discord bot.")
            self.run()

    def run(self):
        try:
            coordinates = self.load_and_clean_data()
            boundary = self.create_polygon_boundary(coordinates)
            base_dir = f"/home/user/abd_ws_2/src/tests/tests/perimeter/{self.perimeter_name}/"

            # === Border Paths ===
            border_paths, _ = generate_border_paths(boundary, offset_distance=self.row_spacing_border, num_levels=2)
            for idx, bp in enumerate(border_paths):
                self.save_waypoints_to_csv(bp, base_dir + f"border_path_{idx}.csv")

            # === Zigzag Paths ===
            primary_waypoints, secondary_waypoints, tertiary_waypoints = self.generate_primary_secondary_tertiary_paths(boundary)
            self.save_waypoints_to_csv(primary_waypoints, base_dir + "primary_zigzag_path_waypoints.csv")
            self.save_waypoints_to_csv(secondary_waypoints, base_dir + "secondary_zigzag_path_waypoints.csv")
            self.save_waypoints_to_csv(tertiary_waypoints, base_dir + "tertiary_zigzag_path_waypoints.csv")

            # === Plot Paths ===
            self.plot_all_paths(boundary, primary_waypoints, secondary_waypoints, tertiary_waypoints, coordinates)

            # === Final Combined CSV with Border + Zigzag ===
            combined_path = []
            for bp in border_paths:
                combined_path.extend(bp)
            combined_path.extend(secondary_waypoints)
            combined_path.extend(reversed(primary_waypoints))
            combined_path.extend(tertiary_waypoints)
            self.save_waypoints_to_csv(combined_path, base_dir + "final_combined_waypoints.csv")

            self.get_logger().info("Three-path zigzag and border generation completed successfully.")
        except Exception as e:
            self.get_logger().error(f"Error in path generation: {e}")

    def load_and_clean_data(self):
        gps_data = pd.read_csv(self.file_path)
        self.get_logger().info("GPS data loaded successfully.")
        gps_data = gps_data.drop_duplicates().dropna()
        latitudes = gps_data['Latitude']
        longitudes = gps_data['Longitude']
        coordinates = list(zip(longitudes, latitudes))
        self.get_logger().info(f"Cleaned GPS data: {len(coordinates)} points")
        return coordinates

    def create_polygon_boundary(self, coordinates):
        polygon = Polygon(coordinates)
        if not polygon.is_valid:
            self.get_logger().warning("Input polygon is invalid. Attempting to fix it.")
            polygon = polygon.buffer(0)
        if isinstance(polygon, MultiPolygon):
            self.get_logger().info("Detected multiple areas. Extracting the largest one.")
            polygon = max(polygon.geoms, key=lambda p: p.area)
        if not polygon.is_valid:
            raise ValueError("Polygon boundary is invalid and could not be fixed.")
        self.get_logger().info("Largest polygon boundary created successfully.")
        return polygon

    def generate_zigzag_path(self, boundary, offset=0.0):
        min_x, min_y, max_x, max_y = boundary.bounds
        waypoints = []
        x_values = np.arange(min_x + offset, max_x, self.row_spacing)
        for i, x in enumerate(x_values):
            line = LineString([(x, min_y), (x, max_y)])
            clipped_line = line.intersection(boundary)
            if not clipped_line.is_empty and clipped_line.geom_type == 'LineString':
                coords = list(clipped_line.coords)
                if i % 2 == 1:
                    coords = coords[::-1]
                waypoints.extend(coords)
        return waypoints

    def generate_primary_secondary_tertiary_paths(self, boundary):
        primary_waypoints = self.generate_zigzag_path(boundary)
        secondary_waypoints = self.generate_zigzag_path(boundary, offset=self.row_spacing / 3)
        tertiary_waypoints = self.generate_zigzag_path(boundary, offset=2 * self.row_spacing / 3)
        return primary_waypoints, secondary_waypoints, tertiary_waypoints

    def save_waypoints_to_csv(self, waypoints, file_name):
        df = pd.DataFrame(waypoints, columns=["longitude", "latitude"])
        df.to_csv(file_name, index=False)
        self.get_logger().info(f"Waypoints saved to {file_name}")

    def plot_all_paths(self, boundary, path1, path2, path3, coordinates):
        plt.figure(figsize=(8, 8))
        plt.plot(*boundary.exterior.xy, color='blue', label='Polygon Boundary')
        x1, y1 = zip(*path1)
        plt.plot(x1, y1, color='orange', label='Primary Zigzag Path')
        x2, y2 = zip(*path2)
        plt.plot(x2, y2, color='green', label='Secondary Zigzag Path')
        x3, y3 = zip(*path3)
        plt.plot(x3, y3, color='purple', label='Tertiary Zigzag Path')
        longitudes, latitudes = zip(*coordinates)
        plt.scatter(longitudes, latitudes, color='red', label='GPS Points')
        plt.title("Primary, Secondary, and Tertiary Zigzag Paths")
        plt.xlabel("Longitude")
        plt.ylabel("Latitude")
        plt.legend()
        plt.grid()
        plt.savefig(self.output_image)
        self.get_logger().info(f"Paths plot saved to {self.output_image}")
        plt.close()

def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
