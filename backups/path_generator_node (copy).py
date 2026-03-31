#!/usr/bin/env python3

import csv
import math
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from shapely.geometry import Point, Polygon, LineString, MultiPolygon
from shapely.affinity import rotate, affine_transform
import rclpy
from rclpy.node import Node

# -------------------------------
# Utility Functions for Rotation
# -------------------------------
def rotate_polygon_to_axis(polygon):
    """
    Rotate the polygon so its minimum bounding rectangle aligns with the x-axis.
    Returns the rotated polygon and the rotation angle (in degrees) used.
    """
    mrr = polygon.minimum_rotated_rectangle
    mrr_coords = list(mrr.exterior.coords)

    # Compute the angle between the first edge and the x-axis.
    dx = mrr_coords[1][0] - mrr_coords[0][0]
    dy = mrr_coords[1][1] - mrr_coords[0][1]
    angle_rad = math.atan2(dy, dx)
    angle_deg = math.degrees(angle_rad)

    # Rotate polygon by -angle_deg around its centroid so that the edge is horizontal.
    rotated = rotate(polygon, -angle_deg, origin='centroid', use_radians=False)
    return rotated, angle_deg

def rotate_back(points, angle_deg, origin):
    """
    Rotate a list of points (tuples) back by angle_deg about the origin.
    Returns a list of transformed points.
    """
    from shapely.affinity import rotate as rotate_geom
    transformed = []
    for p in points:
        pt = Point(p)
        rotated_pt = rotate_geom(pt, angle_deg, origin=origin, use_radians=False)
        transformed.append(tuple(rotated_pt.coords[0]))
    return transformed

def generate_rotated_zigzag(boundary, row_spacing=0.00002, offset=0.0):
    """
    Generate a zigzag path within the given boundary using rotated coordinates.
    The polygon is rotated so that its major axis is aligned with the x-axis,
    zigzag lines are generated, and then the points are rotated back.
    """
    # Rotate polygon to align with x-axis
    rotated, angle_deg = rotate_polygon_to_axis(boundary)
    minx, miny, maxx, maxy = rotated.bounds
    origin = rotated.centroid

    waypoints = []
    # Create zigzag lines spaced by row_spacing with an optional horizontal offset.
    x_values = np.arange(minx + offset, maxx, row_spacing)
    for i, x in enumerate(x_values):
        line = LineString([(x, miny), (x, maxy)])
        clipped = line.intersection(rotated)
        if not clipped.is_empty and clipped.geom_type == 'LineString':
            coords = list(clipped.coords)
            # Alternate direction every row for a continuous zigzag motion.
            if i % 2 == 1:
                coords = coords[::-1]
            waypoints.extend(coords)

    # Rotate the waypoints back to original coordinate frame.
    transformed_waypoints = rotate_back(waypoints, angle_deg, origin)
    return transformed_waypoints

# -------------------------------
# ROS 2 PathGenerator Node
# -------------------------------
class PathGenerator(Node):
    def __init__(self):
        super().__init__('path_generator_node')
        self.get_logger().info("Starting Path Generator Node")

        # File paths and output settings
        self.file_path = "/home/user/abd_ws_2/src/tests/tests/gps_data.csv"  # Input GPS data file path
        self.output_image = "primary_secondary_zigzag_path.png"  # Output image for visualizing paths

        # Parameters for path generation
        self.row_spacing = 0.00003  # Spacing between zigzag lines (adjust as needed) 0.00002 
        self.point_density = 15     # Unused in this version, kept for potential spline smoothing

        # Run the path generation process
        self.run()

    def run(self):
        try:
            # Step 1: Load and clean GPS data
            coordinates = self.load_and_clean_data()

            # Step 2: Create polygon boundary from the GPS data
            boundary = self.create_polygon_boundary(coordinates)

            # Step 3: Generate primary, secondary, and tertiary zigzag paths with different offsets
            primary_waypoints, secondary_waypoints, tertiary_waypoints = self.generate_primary_secondary_tertiary_paths(boundary)

            # Step 4: Save the waypoints for each path to CSV files
            self.save_waypoints_to_csv(primary_waypoints, "primary_zigzag_path_waypoints.csv")
            self.save_waypoints_to_csv(secondary_waypoints, "secondary_zigzag_path_waypoints.csv")
            self.save_waypoints_to_csv(tertiary_waypoints, "tertiary_zigzag_path_waypoints.csv")
            
            # Step 5: Plot and save the boundary and zigzag paths
            self.plot_all_paths(boundary, primary_waypoints, secondary_waypoints, tertiary_waypoints, coordinates)

            # Step 6: Optionally, combine the waypoints from different paths into a single CSV
            self.load_and_concatenate_waypoints("primary_zigzag_path_waypoints.csv",
                                                "secondary_zigzag_path_waypoints.csv",
                                                "tertiary_zigzag_path_waypoints.csv",
                                                "combined_zigzag_path_waypoints.csv")

            self.get_logger().info("Three-path zigzag generation completed successfully.")
        except Exception as e:
            self.get_logger().error(f"Error in path generation: {e}")

    def load_and_clean_data(self):
        """Load GPS data from CSV and perform cleaning (remove duplicates and NaNs)."""
        gps_data = pd.read_csv(self.file_path)
        self.get_logger().info("GPS data loaded successfully.")

        # Remove duplicate entries and any rows with missing data
        gps_data = gps_data.drop_duplicates().dropna()
        latitudes = gps_data['Latitude']
        longitudes = gps_data['Longitude']

        # Combine longitudes and latitudes into coordinate tuples
        coordinates = list(zip(longitudes, latitudes))
        self.get_logger().info(f"Cleaned GPS data: {len(coordinates)} points")
        return coordinates

    def create_polygon_boundary(self, coordinates):
        """
        Create a polygon boundary from GPS coordinates.
        If the polygon is invalid or results in a MultiPolygon,
        select the largest valid polygon.
        """
        polygon = Polygon(coordinates)

        # Check validity and attempt to fix if necessary
        if not polygon.is_valid:
            self.get_logger().warning("Input polygon is invalid. Attempting to fix it.")
            polygon = polygon.buffer(0)

        # Handle MultiPolygon situation by selecting the largest area.
        if isinstance(polygon, MultiPolygon):
            self.get_logger().info("Multiple areas detected. Extracting the largest one.")
            polygon = max(polygon.geoms, key=lambda p: p.area)

        if not polygon.is_valid:
            raise ValueError("Polygon boundary is invalid and could not be fixed.")

        self.get_logger().info("Polygon boundary created successfully.")
        return polygon

    def generate_zigzag_path(self, boundary, offset=0.0):
        """
        Generate a zigzag path using the rotated polygon method.
        The optional offset adjusts the starting position horizontally.
        """
        return generate_rotated_zigzag(boundary, row_spacing=self.row_spacing, offset=offset)

    def generate_primary_secondary_tertiary_paths(self, boundary):
        """
        Generate three sets of waypoints for coverage:
        - Primary path: no offset.
        - Secondary path: offset by one-third of row_spacing.
        - Tertiary path: offset by two-thirds of row_spacing.
        """
        primary_waypoints = self.generate_zigzag_path(boundary, offset=0.0)
        secondary_waypoints = self.generate_zigzag_path(boundary, offset=self.row_spacing / 3)
        tertiary_waypoints = self.generate_zigzag_path(boundary, offset=2 * self.row_spacing / 3)
        return primary_waypoints, secondary_waypoints, tertiary_waypoints

    def save_waypoints_to_csv(self, waypoints, file_name):
        """Save the list of waypoints to a CSV file."""
        df = pd.DataFrame(waypoints, columns=["longitude", "latitude"])
        df.to_csv(file_name, index=False)
        self.get_logger().info(f"Waypoints saved to {file_name}")

    def plot_all_paths(self, boundary, path1, path2, path3, coordinates):
        """Plot the polygon boundary, the three zigzag paths, and the original GPS data."""
        plt.figure(figsize=(8, 8))

        # Plot the polygon boundary
        plt.plot(*boundary.exterior.xy, color='blue', label='Polygon Boundary')

        # Plot primary zigzag path
        x1, y1 = zip(*path1)
        plt.plot(x1, y1, color='orange', label='Primary Zigzag Path')

        # Plot secondary zigzag path
        x2, y2 = zip(*path2)
        plt.plot(x2, y2, color='green', label='Secondary Zigzag Path')

        # Plot tertiary zigzag path
        x3, y3 = zip(*path3)
        plt.plot(x3, y3, color='purple', label='Tertiary Zigzag Path')

        # Plot the original GPS data points
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

    def load_and_concatenate_waypoints(self, primary_file, secondary_file, tertiary_file, output_file):
        """
        Load waypoints from separate CSV files, combine them into one
        ordered list, and save the combined waypoints to a new CSV file.
        """
        waypoints = []
        try:
            # Load secondary waypoints (keep original order)
            with open(secondary_file, 'r') as file:
                reader = csv.reader(file)
                for row_number, row in enumerate(reader, start=1):
                    try:
                        lat = float(row[1])
                        lon = float(row[0])
                        waypoints.append((lon, lat))
                    except (ValueError, IndexError):
                        self.get_logger().warn(f"Skipping invalid row in {secondary_file}, row {row_number}: {row}")

            # Load primary waypoints in reverse order
            primary_waypoints = []
            with open(primary_file, 'r') as file:
                reader = csv.reader(file)
                for row_number, row in enumerate(reader, start=1):
                    try:
                        lat = float(row[1])
                        lon = float(row[0])
                        primary_waypoints.insert(0, (lon, lat))  # Prepend to reverse order
                    except (ValueError, IndexError):
                        self.get_logger().warn(f"Skipping invalid row in {primary_file}, row {row_number}: {row}")

            # Add primary waypoints to the list
            waypoints.extend(primary_waypoints)

            # Load tertiary waypoints (keep original order)
            with open(tertiary_file, 'r') as file:
                reader = csv.reader(file)
                for row_number, row in enumerate(reader, start=1):
                    try:
                        lat = float(row[1])
                        lon = float(row[0])
                        waypoints.append((lon, lat))
                    except (ValueError, IndexError):
                        self.get_logger().warn(f"Skipping invalid row in {tertiary_file}, row {row_number}: {row}")

            # Save the combined waypoints to CSV
            if output_file:
                try:
                    with open(output_file, mode='w', newline='') as file:
                        writer = csv.writer(file)
                        writer.writerow(["longitude", "latitude"])  # CSV header
                        writer.writerows(waypoints)
                    self.get_logger().info(f"Combined waypoints successfully saved to {output_file}")
                except Exception as e:
                    self.get_logger().error(f"An error occurred while saving to CSV: {e}")

        except FileNotFoundError as e:
            self.get_logger().error(f"Waypoint file not found: {e}")
            exit()
        return waypoints

# -------------------------------
# Main entry point for the node
# -------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
