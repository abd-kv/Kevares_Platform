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
# Conversion Functions
# -------------------------------

def latlon_to_xy(lat, lon, ref_lat, ref_lon):
    """Convert lat/lon to local X/Y in meters relative to reference point."""
    R = 6378137  # Earth radius in meters
    d_lat = math.radians(lat - ref_lat)
    d_lon = math.radians(lon - ref_lon)
    mean_lat = math.radians((lat + ref_lat) / 2)

    x = R * d_lon * math.cos(mean_lat)
    y = R * d_lat
    return x, y


def xy_to_latlon(x, y, ref_lat, ref_lon):
    """Convert local X/Y in meters back to lat/lon."""
    R = 6378137
    d_lat = y / R
    d_lon = x / (R * math.cos(math.radians(ref_lat)))

    lat = ref_lat + math.degrees(d_lat)
    lon = ref_lon + math.degrees(d_lon)
    return lat, lon


# -------------------------------
# Polygon Rotation Utilities
# -------------------------------

def rotate_polygon_to_axis(polygon):
    coords = list(polygon.exterior.coords)

    if len(coords) <= 10:
        raise ValueError("Polygon has fewer than 11 points.")

    p_start = coords[0]
    p_end = coords[200]  # not 300, in XY space the 10th is fine

    dx = p_end[0] - p_start[0]
    dy = p_end[1] - p_start[1]

    angle_rad = math.atan2(dx, dy)
    angle_deg = math.degrees(angle_rad)

    print(f"🔁 Rotating polygon by {angle_deg:.2f}° to align 5→10 with Y-axis")

    rotated = shapely_rotate(polygon, angle_deg, origin='centroid', use_radians=False)
    return rotated, -angle_deg  # inverse for rotating back


def rotate_back(points, angle_deg, origin):
    from shapely.affinity import rotate as rotate_geom
    return [rotate_geom(Point(p), angle_deg, origin=origin, use_radians=False).coords[0] for p in points]


# -------------------------------
# Zigzag Path Generator
# -------------------------------

def generate_rotated_zigzag(boundary, row_spacing=0.5, offset=0.0):
    rotated, angle_deg = rotate_polygon_to_axis(boundary)
    minx, miny, maxx, maxy = rotated.bounds
    origin = rotated.centroid
    waypoints = []
    x_values = np.arange(minx + offset, maxx, row_spacing)
    for i, x in enumerate(x_values):
        line = LineString([(x, miny - 10), (x, maxy + 10)])  # Extend lines beyond boundary
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

        # ✔️ Spacing in meters now
        self.row_spacing = 0.5  # Zigzag spacing in meters
        self.row_spacing_border = 0.1  # Border spacing in meters
        self.num_border_levels = 2

        self.run()

    def run(self):
        try:
            coordinates_xy = self.load_and_clean_data()
            boundary = self.create_polygon_boundary(coordinates_xy)

            border_paths_xy, inner_boundary = self.generate_border_paths(
                boundary, offset_distance=self.row_spacing_border, num_levels=self.num_border_levels)

            for idx, bp in enumerate(border_paths_xy):
                self.save_waypoints_to_csv(bp, f"border_path_{idx}.csv")

            expanded_boundary = inner_boundary.buffer(0.05)  # Small buffer for zigzag space

            primary = generate_rotated_zigzag(expanded_boundary, row_spacing=self.row_spacing, offset=0.0)
            secondary = generate_rotated_zigzag(expanded_boundary, row_spacing=self.row_spacing, offset=self.row_spacing / 3)
            tertiary = generate_rotated_zigzag(expanded_boundary, row_spacing=self.row_spacing, offset=2 * self.row_spacing / 3)

            self.save_waypoints_to_csv(primary, "primary_zigzag_path.csv")
            self.save_waypoints_to_csv(secondary, "secondary_zigzag_path.csv")
            self.save_waypoints_to_csv(tertiary, "tertiary_zigzag_path.csv")

            self.combine_all_paths(border_paths_xy, primary, secondary, tertiary, "final_combined_waypoints.csv")

            self.plot_all_paths(boundary, border_paths_xy, primary, secondary, tertiary, coordinates_xy)

            self.get_logger().info("✅ Zigzag path generation completed with proper metric spacing.")
        except Exception as e:
            self.get_logger().error(f"❌ Error in path generation: {e}")

    def load_and_clean_data(self):
        gps_data = pd.read_csv(self.file_path)
        self.get_logger().info("GPS data loaded successfully.")
        gps_data = gps_data.drop_duplicates().dropna()

        latitudes = gps_data['Latitude'].tolist()
        longitudes = gps_data['Longitude'].tolist()

        self.ref_lat = latitudes[0]
        self.ref_lon = longitudes[0]

        xy_points = [latlon_to_xy(lat, lon, self.ref_lat, self.ref_lon) for lat, lon in zip(latitudes, longitudes)]
        return xy_points

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

    def generate_border_paths(self, boundary, offset_distance=0.1, num_levels=1):
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

    def save_waypoints_to_csv(self, waypoints_xy, file_name):
        waypoints_latlon = [xy_to_latlon(x, y, self.ref_lat, self.ref_lon) for x, y in waypoints_xy]
        pd.DataFrame(waypoints_latlon, columns=["longitude", "latitude"]).to_csv(file_name, index=False)
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
        bx, by = boundary.exterior.xy
        plt.plot(bx, by, color='navy', linewidth=2, label='Original Boundary')

        border_colors = ['magenta', 'cyan', 'lime']
        for idx, bp in enumerate(border_paths):
            try:
                x, y = zip(*bp)
                plt.plot(x, y, color=border_colors[idx % len(border_colors)], linestyle='--', linewidth=2, label=f'Border Path {idx}')
            except Exception as e:
                self.get_logger().warn(f"Could not plot border path {idx}: {e}")

        if primary:
            x1, y1 = zip(*primary)
            plt.plot(x1, y1, color='red', label='Primary Zigzag')
        if secondary:
            x2, y2 = zip(*secondary)
            plt.plot(x2, y2, color='green', label='Secondary Zigzag')
        if tertiary:
            x3, y3 = zip(*tertiary)
            plt.plot(x3, y3, color='purple', label='Tertiary Zigzag')

        if original_points:
            xg, yg = zip(*original_points)
            plt.scatter(xg, yg, color='gray', s=20, label='GPS Points')

        plt.title("Coverage Paths: Zigzag + Borders")
        plt.xlabel("Meters (Local X)")
        plt.ylabel("Meters (Local Y)")
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
