import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32MultiArray
from cv_bridge import CvBridge
import os
import time  # Import time for timestamp
import json
import cv2
import csv

class DataCaptureNode(Node):
    def __init__(self):
        super().__init__('data_capture_node')

        # Initialize main image directory
        self.image_save_dir = '/home/user/vaughan_pilot_images'
        
        # Initialize variables
        self.bridge = CvBridge()
        self.capturing = False
        self.last_position = 0
        self.current_position = 0
        self.distance_moved = 0
        self.capture_count = 1  # Track capture folder number

        # Initialize placeholders for images, depth, and intrinsics (fx, fy, cx, cy)
        self.left_image = None
        self.right_image = None
        self.left_depth = None
        self.right_depth = None
        self.left_intrinsics = None  # For storing fx, fy, cx, cy of left camera
        self.right_intrinsics = None  # For storing fx, fy, cx, cy of right camera

        # Initialize GPS placeholders
        self.latitude = None
        self.longitude = None
        self.altitude = None

        # Create subscribers for RealSense images, depth, camera info (as Int32MultiArray), and CAN feedback
        self.create_subscription(Image, 'rear_left_color_image', self.rear_left_image_callback, 10)
        self.create_subscription(Image, 'rear_right_color_image', self.rear_right_image_callback, 10)
        self.create_subscription(Image, 'rear_left_depth_image', self.rear_left_depth_callback, 10)
        self.create_subscription(Image, 'rear_right_depth_image', self.rear_right_depth_callback, 10)
        self.create_subscription(Int32MultiArray, 'rear_left_camera_info_array', self.left_camera_info_callback, 10)
        self.create_subscription(Int32MultiArray, 'rear_right_camera_info_array', self.right_camera_info_callback, 10)
        self.create_subscription(String, 'can_feedback', self.can_feedback_callback, 10)
        self.create_subscription(String, 'gps_data', self.gps_callback, 10)  # GPS data subscription

    # Callbacks for image topics
    def rear_left_image_callback(self, msg):
        self.left_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def rear_right_image_callback(self, msg):
        self.right_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def rear_left_depth_callback(self, msg):
        self.left_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')

    def rear_right_depth_callback(self, msg):
        self.right_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')

    # Callbacks for Int32MultiArray (camera intrinsics)
    def left_camera_info_callback(self, msg):
        self.left_intrinsics = msg.data  # fx, fy, cx, cy

    def right_camera_info_callback(self, msg):
        self.right_intrinsics = msg.data  # fx, fy, cx, cy

    # Callback for GPS data (JSON string containing latitude, longitude, altitude)
    def gps_callback(self, msg):
        gps_data = json.loads(msg.data)
        self.latitude = gps_data.get("latitude")
        self.longitude = gps_data.get("longitude")
        self.altitude = gps_data.get("altitude")

    def create_new_folder(self):
        timestamp = int(round(time.time() * 10))
        """ Create new folder structure and CSV files for a new capture session. """
        capture_folder_color = os.path.join(self.image_save_dir, f'capture_{timestamp}', 'color')
        capture_folder_depth = os.path.join(self.image_save_dir, f'capture_{timestamp}', 'depth')
        self.left_image_dir_color = os.path.join(capture_folder_color, 'rear_left')
        self.right_image_dir_color = os.path.join(capture_folder_color, 'rear_right')
        self.left_image_dir_depth = os.path.join(capture_folder_depth, 'rear_left')
        self.right_image_dir_depth = os.path.join(capture_folder_depth, 'rear_right')

        # Create directories for left and right images
        os.makedirs(self.left_image_dir_color, exist_ok=True)
        os.makedirs(self.right_image_dir_color, exist_ok=True)
        os.makedirs(self.left_image_dir_depth, exist_ok=True)
        os.makedirs(self.right_image_dir_depth, exist_ok=True)

        # Create and initialize CSV files for storing image and GPS data
        
        self.left_csv_path = os.path.join(self.image_save_dir, f'capture_{timestamp}', 'rear_left.csv')
        self.right_csv_path = os.path.join(self.image_save_dir, f'capture_{timestamp}', 'rear_right.csv')
        self.gps_csv_path = os.path.join(self.image_save_dir, f'capture_{timestamp}', 'gps_data.csv')

        # Write CSV headers for both CSV files
        with open(self.left_csv_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["image_name", "fx", "fy", "cx", "cy"])

        with open(self.right_csv_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["image_name", "fx", "fy", "cx", "cy"])

        # Write CSV header for GPS data
        with open(self.gps_csv_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["timestamp", "latitude", "longitude", "altitude"])

        self.get_logger().info(f"New directories and CSVs created for capture {self.capture_count}")

    def capture_images(self):
        timestamp = int(round(time.time() * 10))  # Use time.time() for the timestamp

        # Define file paths for left and right images with timestamp
        left_image_path_color = os.path.join(self.left_image_dir_color, f'{timestamp}.png')
        right_image_path_color = os.path.join(self.right_image_dir_color, f'{timestamp}.png')
        left_image_path_depth = os.path.join(self.left_image_dir_depth, f'{timestamp}.png')
        right_image_path_depth = os.path.join(self.right_image_dir_depth, f'{timestamp}.png')

        # Save and log the left image
        if self.left_image is not None and self.left_intrinsics is not None:
            cv2.imwrite(left_image_path_color, self.left_image)
            cv2.imwrite(left_image_path_depth, self.left_depth)
            self.get_logger().info(f'Captured and saved left image in {left_image_path_color}')
            
            # Append to the rear_left.csv
            with open(self.left_csv_path, mode='a', newline='') as file:
                writer = csv.writer(file)
                # Write the intrinsic data (fx, fy, cx, cy)
                writer.writerow([
                    left_image_path_color,
                    self.left_intrinsics[0],  # fx
                    self.left_intrinsics[1],  # fy
                    self.left_intrinsics[2],  # cx
                    self.left_intrinsics[3]   # cy
                ])
        else:
            self.get_logger().warn("Left image or CameraInfo (intrinsics) is not available.")

        # Save and log the right image
        if self.right_image is not None and self.right_intrinsics is not None:
            cv2.imwrite(right_image_path_color, self.right_image)
            cv2.imwrite(right_image_path_depth, self.right_depth)
            self.get_logger().info(f'Captured and saved right image in {right_image_path_color}')
            
            # Append to the rear_right.csv
            with open(self.right_csv_path, mode='a', newline='') as file:
                writer = csv.writer(file)
                # Write the intrinsic data (fx, fy, cx, cy)
                writer.writerow([
                    right_image_path_color,
                    self.right_intrinsics[0],  # fx
                    self.right_intrinsics[1],  # fy
                    self.right_intrinsics[2],  # cx
                    self.right_intrinsics[3]   # cy
                ])
        else:
            self.get_logger().warn("Right image or CameraInfo (intrinsics) is not available.")

        # Save GPS data to the CSV file
        if self.latitude is not None and self.longitude is not None:
            with open(self.gps_csv_path, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([timestamp, self.latitude, self.longitude, self.altitude])
            self.get_logger().info(f"GPS data saved with timestamp: {timestamp}")
        else:
            self.get_logger().warn("No valid GPS data available to save.")

    def can_feedback_callback(self, msg):
        try:
            data = json.loads(msg.data)
            arbitration_id = data.get("id")
            
            if arbitration_id == "0x311":
                left_wheel_mileometer = data["left_wheel_mileometer"]
                right_wheel_mileometer = data["right_wheel_mileometer"]
                self.current_position = (left_wheel_mileometer + right_wheel_mileometer) / 2
                self.distance_moved = self.current_position - self.last_position
                self.get_logger().info(f"Current position: {self.current_position}, Distance moved: {self.distance_moved}")

                # Capture images if the rover has moved at least 1000 mm
                if self.distance_moved >= 610 and self.capturing:
                    self.capture_images()
                    self.last_position = self.current_position  # Update last position

            elif arbitration_id == "0x241":
                swd = data.get("swd")  # Extract switch value
                if swd == 1 and not self.capturing: 
                    self.capturing = True
                    self.create_new_folder()
                    self.get_logger().info("Started capturing images.")
                elif swd == 0 and self.capturing:
                    self.capturing = False
                    self.capture_count += 1  # Increment capture count for next folder
                    self.get_logger().info("Stopped capturing images.")
        except Exception as e:
            self.get_logger().error(f"Error processing CAN data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DataCaptureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Script terminated by user.")
    finally:
        node.destroy
