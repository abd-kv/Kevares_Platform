#!/usr/bin/env python3

import csv
import math
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from shapely.geometry import Point, Polygon, LineString, MultiPolygon
from shapely.affinity import rotate as shapely_rotate
import rclpy
from rclpy.node import Node

# -------------------------------
# Rotate polygon so line 5→10 aligns with Y-axis
# -------------------------------
def rotate_polygon_to_axis(polygon):
    coords = list(polygon.exterior.coords)

    if len(coords) <= 10:
        raise ValueError("Polygon has fewer than 11 points.")

    p_start = coords[0]*111000  # (lon, lat)
    p_end = coords[300]*111000  # (lon, lat)
    print(len(coords))
    # Adjust longitude for latitude curvature
    x_rad = math.radians(p_start[1])
    dx = (p_start[0] - p_end[0])/ math.cos(x_rad)
    dy = (p_start[1] - p_end[1]) 
    # Angle from X-axis
    angle_rad = math.atan2(dx, dy)
    angle_deg = math.degrees(angle_rad)

    # Rotate polygon so that line aligns with Y-axis
    rotation_deg = angle_deg
    print(x_rad)
    print(f"🔁 Rotating polygon by {rotation_deg:.2f}° to align 5→10 with Y-axis")

    rotated = shapely_rotate(polygon, rotation_deg, origin='centroid', use_radians=False)
    return rotated, -rotation_deg  # use inverse for rotating back

def rotate_back(points, angle_deg, origin):
    from shapely.affinity import rotate as rotate_geom
    return [rotate_geom(Point(p), angle_deg, origin=origin, use_radians=False).coords[0] for p in points]

def generate_rotated_zigzag(boundary, row_spacing=0.00002, offset=0.0):
    rotated, angle_deg = rotate_polygon_to_axis(boundary)
    minx, miny, maxx, maxy = rotated.bounds
    origin = rotated.centroid
    waypoints = []
    x_values = np.arange(minx + offset, maxx, row_spacing)
    for i, x in enumerate(x_values):
        line = LineString([(x, miny - 0.01), (x, maxy + 0.01)])  # Extend lines
        clipped = line.intersection(rotated)
        if not clipped.is_empty and clipped.geom_type == 'LineString':
            coords = list(clipped.coords)
            if i % 2 == 1:
                coords = coords[::-1]
            waypoints.extend(coords)
    return rotate_back(waypoints, angle_deg, origin)

# -------------------------------
# Main Node
# -------------------------------
class PathGenerator(Node):
    def __init__(self):
        super().__init__('path_generator_node')
        self.get_logger().info("Starting Path Generator Node")
        self.file_path = "/home/user/abd_ws_2/src/tests/tests/perimeter/23JunC/gps_data.csv"
        self.output_image = "coverage_paths.png"
        self.row_spacing = 0.00004
        self.row_spacing_border = 0.000005
        self.num_border_levels = 2
        self.run()

    def run(self):
        try:
            coordinates = self.load_and_clean_data()
            boundary = self.create_polygon_boundary(coordinates)
            border_paths, inner_boundary = self.generate_border_paths(
                boundary, offset_distance=self.row_spacing_border, num_levels=self.num_border_levels)
            for idx, bp in enumerate(border_paths):
                self.save_waypoints_to_csv(bp, f"border_path_{idx}.csv")

            expanded_boundary = inner_boundary.buffer(0.000005)
            primary = generate_rotated_zigzag(expanded_boundary, row_spacing=self.row_spacing, offset=0.0)
            secondary = generate_rotated_zigzag(expanded_boundary, row_spacing=self.row_spacing, offset=self.row_spacing / 3)
            tertiary = generate_rotated_zigzag(expanded_boundary, row_spacing=self.row_spacing, offset=2 * self.row_spacing / 3)

            self.save_waypoints_to_csv(primary, "primary_zigzag_path_waypoints.csv")
            self.save_waypoints_to_csv(secondary, "secondary_zigzag_path_waypoints.csv")
            self.save_waypoints_to_csv(tertiary, "tertiary_zigzag_path_waypoints.csv")

            self.combine_all_paths(border_paths, primary, secondary, tertiary, "final_combined_waypoints.csv")
            self.plot_all_paths(boundary, border_paths, primary, secondary, tertiary, coordinates)

            self.get_logger().info("✅ Zigzag path generation (Y-axis aligned) completed.")
        except Exception as e:
            self.get_logger().error(f"❌ Error in path generation: {e}")

    def load_and_clean_data(self):
        gps_data = pd.read_csv(self.file_path)
        self.get_logger().info("GPS data loaded successfully.")
        gps_data = gps_data.drop_duplicates().dropna()
        latitudes = gps_data['Latitude']
        longitudes = gps_data['Longitude']
        return list(zip(longitudes, latitudes))

    def create_polygon_boundary(self, coordinates):
        polygon = Polygon(coordinates)
        if not polygon.is_valid:
            self.get_logger().warning("Input polygon is invalid. Attempting to fix it.")
            polygon = polygon.buffer(0)
        if isinstance(polygon, MultiPolygon):
            polygon = max(polygon.geoms, key=lambda p: p.area)
        if not polygon.is_valid:
            raise ValueError("Polygon boundary is invalid and could not be fixed.")
        return polygon

    def generate_border_paths(self, boundary, offset_distance=0.00001, num_levels=1):
        border_paths = []
        current_boundary = boundary
        for level in range(num_levels + 1):
            border_paths.append(list(current_boundary.exterior.coords))
            new_boundary = current_boundary.buffer(-offset_distance)
            if new_boundary.is_empty:
                break
            if new_boundary.geom_type == 'MultiPolygon':
                new_boundary = max(new_boundary.geoms, key=lambda p: p.area)
            current_boundary = new_boundary
        return border_paths, current_boundary

    def save_waypoints_to_csv(self, waypoints, file_name):
        pd.DataFrame(waypoints, columns=["longitude", "latitude"]).to_csv(file_name, index=False)
        self.get_logger().info(f"Waypoints saved to {file_name}")

    def combine_all_paths(self, border_paths, primary, secondary, tertiary, output_file):
        combined = []
        for bp in border_paths:
            combined.extend(bp)
        combined.extend(secondary)
        combined.extend(list(reversed(primary)))
        combined.extend(tertiary)
        self.save_waypoints_to_csv(combined, output_file)

    def plot_all_paths(self, boundary, border_paths, primary, secondary, tertiary, original_points):
        plt.figure(figsize=(10, 10))
        plt.plot(*boundary.exterior.xy, color='navy', linewidth=2, label='Original Boundary')
        border_colors = ['magenta', 'cyan', 'lime']
        for idx, bp in enumerate(border_paths):
            try:
                x, y = zip(*bp)
                plt.plot(x, y, color=border_colors[idx % len(border_colors)], linestyle='--', linewidth=2, label=f'Border Path {idx}')
            except Exception as e:
                self.get_logger().warn(f"Could not plot border path {idx}: {e}")
        if primary:
            try:
                x1, y1 = zip(*primary)
                plt.plot(x1, y1, color='red', label='Primary Zigzag Path')
            except ValueError:
                self.get_logger().warn("Primary zigzag path is empty. Skipping plot.")
        if secondary:
            try:
                x2, y2 = zip(*secondary)
                plt.plot(x2, y2, color='green', label='Secondary Zigzag Path')
            except ValueError:
                self.get_logger().warn("Secondary zigzag path is empty. Skipping plot.")
        if tertiary:
            try:
                x3, y3 = zip(*tertiary)
                plt.plot(x3, y3, color='purple', label='Tertiary Zigzag Path')
            except ValueError:
                self.get_logger().warn("Tertiary zigzag path is empty. Skipping plot.")
        if original_points:
            xg, yg = zip(*original_points)
            plt.scatter(xg, yg, color='gray', s=20, label='GPS Points')
        plt.title("Coverage Paths: Zigzag Aligned to Segment 5→10 (Y-axis)")
        plt.xlabel("Longitude")
        plt.ylabel("Latitude")
        plt.legend()
        plt.grid(True)
        plt.savefig(self.output_image)
        self.get_logger().info(f"🖼️ Coverage paths plot saved to {self.output_image}")
        plt.close()

# -------------------------------
# Entry Point
# -------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
