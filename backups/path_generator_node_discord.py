#!/usr/bin/env python3

import csv
import math
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from shapely.geometry import Point, Polygon, LineString, MultiPolygon
from shapely.affinity import rotate
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os

# -------------------------------
# Utility Functions for Rotation
# -------------------------------
def rotate_polygon_to_axis(polygon):
    mrr = polygon.minimum_rotated_rectangle
    mrr_coords = list(mrr.exterior.coords)
    dx = (mrr_coords[1][0] - mrr_coords[0][0])/ math.cos(math.radians(mrr_coords[0][0]))
    dy = mrr_coords[1][1] - mrr_coords[0][1]
    angle_rad = math.atan2(dy, dx)
    angle_deg = math.degrees(angle_rad)
    rotated = rotate(polygon, angle_deg-5, origin='centroid', use_radians=False)
    return rotated, angle_deg

def rotate_back(points, angle_deg, origin):
    from shapely.affinity import rotate as rotate_geom
    transformed = []
    for p in points:
        pt = Point(p)
        rotated_pt = rotate_geom(pt, angle_deg, origin=origin, use_radians=False)
        transformed.append(tuple(rotated_pt.coords[0]))
    return transformed

def generate_rotated_zigzag(boundary, row_spacing=0.00002, offset=0.0):
    rotated, angle_deg = rotate_polygon_to_axis(boundary)
    minx, miny, maxx, maxy = rotated.bounds
    origin = rotated.centroid

    waypoints = []
    x_values = np.arange(minx + offset, maxx, row_spacing)
    for i, x in enumerate(x_values):
        line = LineString([(x, miny), (x, maxy)])
        clipped = line.intersection(rotated)
        if not clipped.is_empty and clipped.geom_type == 'LineString':
            coords = list(clipped.coords)
            if i % 2 == 1:
                coords = coords[::-1]
            waypoints.extend(coords)
    transformed_waypoints = rotate_back(waypoints, angle_deg, origin)
    return transformed_waypoints

def generate_border_paths(boundary, offset_distance=0.00001, num_levels=1):
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
        self.get_logger().info("🧩 Path Generator Node initialized. Waiting for perimeter name...")

        self.perimeter_dir = "/home/user/abd_ws_2/src/tests/tests/perimeter"
        self.perimeter_name = "default"
        self.file_path = None

        self.row_spacing = 0.00004
        self.row_spacing_border = 0.000005
        self.num_border_levels = 2

        self.name_sub = self.create_subscription(String, '/perimeter_name', self.name_callback, 10)

    def name_callback(self, msg):
        self.perimeter_name = msg.data.strip()
        self.file_path = os.path.join(self.perimeter_dir, self.perimeter_name, "gps_data.csv")
        self.output_image = os.path.join(self.perimeter_dir, self.perimeter_name, "coverage_paths.png")

        if not os.path.exists(self.file_path):
            self.get_logger().error(f"❌ GPS data not found: {self.file_path}")
            return

        try:
            coordinates = self.load_and_clean_data()
            boundary = self.create_polygon_boundary(coordinates)

            border_paths, inner_boundary = generate_border_paths(
                boundary, offset_distance=self.row_spacing_border, num_levels=self.num_border_levels
            )

            for idx, bp in enumerate(border_paths):
                self.save_waypoints_to_csv(bp, os.path.join(self.perimeter_dir, self.perimeter_name, f"border_path_{idx}.csv"))

            if not inner_boundary.is_empty:
                expanded_boundary = inner_boundary.buffer(0.000005)
                primary_waypoints = generate_rotated_zigzag(expanded_boundary, row_spacing=self.row_spacing, offset=0.0)
                secondary_waypoints = generate_rotated_zigzag(expanded_boundary, row_spacing=self.row_spacing, offset=self.row_spacing / 3)
                tertiary_waypoints = generate_rotated_zigzag(expanded_boundary, row_spacing=self.row_spacing, offset=2 * self.row_spacing / 3)

                self.save_waypoints_to_csv(primary_waypoints, os.path.join(self.perimeter_dir, self.perimeter_name, "primary_zigzag_path_waypoints.csv"))
                self.save_waypoints_to_csv(secondary_waypoints, os.path.join(self.perimeter_dir, self.perimeter_name, "secondary_zigzag_path_waypoints.csv"))
                self.save_waypoints_to_csv(tertiary_waypoints, os.path.join(self.perimeter_dir, self.perimeter_name, "tertiary_zigzag_path_waypoints.csv"))
            else:
                self.get_logger().warn("No inner area remaining for zigzag path generation.")
                primary_waypoints = secondary_waypoints = tertiary_waypoints = []

            self.combine_all_paths(border_paths, primary_waypoints, secondary_waypoints, tertiary_waypoints, 
                                   os.path.join(self.perimeter_dir, self.perimeter_name, "final_combined_waypoints.csv"))

            self.plot_all_paths(boundary, border_paths, primary_waypoints, secondary_waypoints, tertiary_waypoints, coordinates)
            self.get_logger().info("✅ Path generation completed successfully.")

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
            polygon = max(polygon.geoms, key=lambda p: p.area)
        if not polygon.is_valid:
            raise ValueError("Polygon boundary is invalid and could not be fixed.")
        self.get_logger().info("Polygon boundary created successfully.")
        return polygon

    def save_waypoints_to_csv(self, waypoints, file_name):
        df = pd.DataFrame(waypoints, columns=["longitude", "latitude"])
        df.to_csv(file_name, index=False)
        self.get_logger().info(f"Waypoints saved to {file_name}")

    def combine_all_paths(self, border_paths, primary, secondary, tertiary, output_file):
        combined_waypoints = []
        for bp in border_paths:
            combined_waypoints.extend(bp)
        combined_waypoints.extend(secondary)
        combined_waypoints.extend(list(reversed(primary)))
        combined_waypoints.extend(tertiary)
        self.save_waypoints_to_csv(combined_waypoints, output_file)
        self.get_logger().info(f"Final combined waypoints saved to {output_file}")

    def plot_all_paths(self, boundary, border_paths, primary, secondary, tertiary, original_points):
        plt.figure(figsize=(10, 10))
        plt.plot(*boundary.exterior.xy, color='navy', linewidth=2, label='Original Boundary')

        border_colors = ['magenta', 'cyan', 'lime', 'orange']
        for idx, bp in enumerate(border_paths):
            try:
                x, y = zip(*bp)
                plt.plot(x, y, color=border_colors[idx % len(border_colors)], linestyle='--',
                         linewidth=2, label=f'Border Path {idx}')
            except Exception as e:
                self.get_logger().warn(f"Could not plot border path {idx}: {e}")
        if primary:
            xp, yp = zip(*primary)
            plt.plot(xp, yp, color='red', linewidth=2, label='Primary Zigzag Path')
        if secondary:
            xs, ys = zip(*secondary)
            plt.plot(xs, ys, color='green', linewidth=2, label='Secondary Zigzag Path')
        if tertiary:
            xt, yt = zip(*tertiary)
            plt.plot(xt, yt, color='purple', linewidth=2, label='Tertiary Zigzag Path')

        longitudes, latitudes = zip(*original_points)
        plt.scatter(longitudes, latitudes, color='gray', marker='o', s=20, label='GPS Points')

        plt.title("Coverage Paths: Border and 3 Zigzag Paths")
        plt.xlabel("Longitude")
        plt.ylabel("Latitude")
        plt.legend(loc='best')
        plt.grid(True)
        plt.savefig(self.output_image)
        self.get_logger().info(f"Coverage paths plot saved to {self.output_image}")
        plt.close()

def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🔌 Path generator interrupted. Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
