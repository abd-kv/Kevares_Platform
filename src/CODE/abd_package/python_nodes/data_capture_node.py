import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge
from datetime import datetime
import os
import json

class DataCaptureNode(Node):
    def __init__(self):
        super().__init__('data_capture_node')

        self.subscription_can = self.create_subscription(
            String,
            'can_feedback',
            self.listener_callback_can,
            10)
        self.subscription_image = self.create_subscription(
            Image,
            'color_image',
            self.listener_callback_image,   
            10)
        self.subscription_can  # prevent unused variable warning
        self.subscription_image  # prevent unused variable warning

        self.bridge = CvBridge()
        self.capturing = False
        self.current_image_folder = ""
        self.image_folder_base = "/media/himanshu/extra/captured_images" #/home/adelaide-mkiii/
        self.color_image = None

        self.get_logger().info("Data Capture node started")

    def create_new_folder(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.current_image_folder = os.path.join(self.image_folder_base, f"images_{timestamp}")
        os.makedirs(self.current_image_folder, exist_ok=True)
        self.get_logger().info(f"Created new folder: {self.current_image_folder}")
        return

    def capture_image(self):
        if self.color_image is None:
            self.get_logger().info("No color image available.")
            return

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        image_path = os.path.join(self.current_image_folder, f"image_{timestamp}.jpg")
        cv2.imwrite(image_path, self.color_image)
        self.get_logger().info(f"Saved image: {image_path}")
        return

    def listener_callback_image(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def listener_callback_can(self, msg):
        self.get_logger().info(f"Received CAN data: {msg.data}")

        try:
            data = json.loads(msg.data)
            arbitration_id = data.get("id")
            if arbitration_id == "0x241":
                swd = data.get("swd")
                self.get_logger().info(f"swd: {swd}")
                if swd == 1 and not self.capturing: 
                    self.capturing = True
                    self.create_new_folder()
                    self.get_logger().info("Started capturing images.")
                elif swd == 0 and self.capturing:
                    self.capturing = False
                    self.get_logger().info("Stopped capturing images.")

                if self.capturing:
                    self.capture_image()

        except Exception as e:
            self.get_logger().error(f"Error processing CAN data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DataCaptureNode()
    try:
        rclpy.spin(node)  # Keeps the node running
    except KeyboardInterrupt:
        node.get_logger().info("Script terminated by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
