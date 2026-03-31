import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import os
import numpy as np
from PIL import Image as PILImage
import datetime

class SegmentationNode(Node):
    def __init__(self):
        super().__init__('segmentation_node')

        # Load YOLOv8 model for segmentation
        self.model = YOLO(r'/home/user/ros2_ws/src/vaughan_pilot/vaughan_pilot/best.pt') 

        # Initialize variables
        self.bridge = CvBridge()

        # Subscribe to the captured images topics
        self.create_subscription(Image, 'captured_left_image', self.left_image_callback, 10)
        self.create_subscription(Image, 'captured_right_image', self.right_image_callback, 10)

        # Define directories for saving the segmented images and masks
        self.left_prediction_dir = '/path/to/save/left/predictions'
        self.left_mask_dir = '/path/to/save/left/masks'
        self.right_prediction_dir = '/path/to/save/right/predictions'
        self.right_mask_dir = '/path/to/save/right/masks'

        # Create directories if they don't exist
        os.makedirs(self.left_prediction_dir, exist_ok=True)
        os.makedirs(self.left_mask_dir, exist_ok=True)
        os.makedirs(self.right_prediction_dir, exist_ok=True)
        os.makedirs(self.right_mask_dir, exist_ok=True)

    def left_image_callback(self, msg):
        self.process_and_save_image(msg, 'left')

    def right_image_callback(self, msg):
        self.process_and_save_image(msg, 'right')

    def process_and_save_image(self, msg, camera_position):
        # Convert ROS Image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Create timestamp for unique filenames
        timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')

        # Make predictions on the image using YOLOv8
        results = self.model.predict(source=cv_image)

        # Set directories based on camera position
        if camera_position == 'left':
            prediction_dir = self.left_prediction_dir
            mask_dir = self.left_mask_dir
        else:
            prediction_dir = self.right_prediction_dir
            mask_dir = self.right_mask_dir

        # Iterate through results to save prediction images and masks
        for i, result in enumerate(results):
            # Save the prediction image
            output_prediction_path = os.path.join(prediction_dir, f'{timestamp}_prediction_{i}.jpg')
            result_img = PILImage.fromarray(result.plot()).convert('RGB')
            result_img.save(output_prediction_path)

            # Log the saved prediction
            self.get_logger().info(f'Segmented {camera_position} image saved at {output_prediction_path}')

            # Handle mask saving if available
            if result.masks is not None:
                mask_data = result.masks.data.cpu().numpy()

                # Create an empty mask to combine all object masks
                combined_mask = np.zeros(mask_data[0].shape, dtype=np.uint8)

                # Combine all masks using a logical OR operation
                for mask in mask_data:
                    combined_mask = np.logical_or(combined_mask, mask).astype(np.uint8)

                # Convert the combined mask to a binary mask (0 and 255)
                combined_mask_img = PILImage.fromarray((combined_mask * 255).astype(np.uint8))

                # Save the combined mask image
                output_mask_path = os.path.join(mask_dir, f'{timestamp}_combined_mask.png')
                combined_mask_img.save(output_mask_path)

                # Log the saved mask
                self.get_logger().info(f'Combined mask for {camera_position} image saved at {output_mask_path}')

def main(args=None):
    rclpy.init(args=args)
    node = SegmentationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Segmentation node terminated by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

