import os
import time
import cv2
import numpy as np
from PIL import Image as PILImage
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

class YOLOSegmentationNode(Node):
    def __init__(self):
        super().__init__('mask_creation_node')

        # Initialize the directory paths
        self.image_save_dir = '/home/user/vaughan_pilot_images/masks'
        
        # Create a unique capture folder
        self.capture_folder = self.get_next_capture_folder(self.image_save_dir)
        os.makedirs(self.capture_folder, exist_ok=True)

        # Create subfolder for the mask files
        self.mask_dir = os.path.join(self.capture_folder, 'masks')
        os.makedirs(self.mask_dir, exist_ok=True)

        # Load the sidewalk model for prediction
        self.model = YOLO(r'/home/user/ros2_ws/src/vaughan_pilot/vaughan_pilot/Models/epoch70.pt')
        

        # Define class names (adjust based on your model's output)
        self.class_names = ['horizontal_gap', 'vertical_gap', 'pothole', 'crack', 'asphalt_patch', 'epoxy_patch']

        # Create subfolders for each class in the mask directory
        for class_name in self.class_names:
            os.makedirs(os.path.join(self.mask_dir, class_name), exist_ok=True)

        # Initialize ROS2 bridge
        self.bridge = CvBridge()

        # Create a subscription to the image topic (only one camera is being processed now)
        self.create_subscription(Image, 'rear_left_color_image', self.process_image_callback, 10)

    def get_next_capture_folder(self, base_dir):
        """
        Generate a new unique capture folder name (e.g., capture_1, capture_2, ...).
        """
        i = 1
        while True:
            capture_folder = os.path.join(base_dir, f'capture_{i}')
            if not os.path.exists(capture_folder):
                return capture_folder
            i += 1

    def process_image_callback(self, msg):
        """
        This function is called when an image is received from the publisher node.
        """
        # Convert the ROS image message to a CV2 image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert the image to RGB (YOLO uses RGB format)
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        # Run YOLO segmentation on the image
        results = self.model.predict(source=rgb_image)

        # Get the timestamp for saving the files
        timestamp = int(round(time.time() * 10))  # Using time.time() for the timestamp

        for i, result in enumerate(results):
            # Save the prediction image
            output_image_path = os.path.join(self.capture_folder, f'prediction_{timestamp}_{i}.jpg')
            result_img = PILImage.fromarray(result.plot()).convert('RGB')
            result_img.save(output_image_path)

            # Process the masks
            if result.masks is not None:
                masks_data = result.masks.data.cpu().numpy()  # Get the mask data
                boxes = result.boxes  # Access bounding boxes for class information

                class_ids = boxes.cls.cpu().numpy().astype(int)  # Get the class IDs

                # Iterate over each mask and save it in the corresponding class folder
                for class_id, mask in zip(class_ids, masks_data):
                    if class_id < len(self.class_names):  # Ensure valid class index
                        binary_mask = (mask > 0).astype(np.uint8) * 255  # Convert to binary mask

                        # Convert the binary mask to a PIL image
                        mask_img = PILImage.fromarray(binary_mask)

                        # Save the mask in the correct class folder
                        output_mask_path = os.path.join(self.mask_dir, self.class_names[class_id], f'{timestamp}_{i}.png')
                        mask_img.save(output_mask_path)

                        self.get_logger().info(f"Mask saved for class {self.class_names[class_id]}: {output_mask_path}")

def main(args=None):
    rclpy.init(args=args)
    node = YOLOSegmentationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Segmentation node interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
