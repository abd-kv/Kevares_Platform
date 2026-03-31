import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np

class RearRightRealSensePublisher(Node):

    def __init__(self):
        super().__init__('right_realsense_publisher_node')
        self.publisher_color = self.create_publisher(Image, 'right_color_image', 10)
        self.publisher_depth = self.create_publisher(Image, 'right_depth_image', 10)
        self.publisher_intrinsics = self.create_publisher(Int32MultiArray, 'right_camera_info_array', 10)
        self.bridge = CvBridge()

        # Initialize RealSense pipeline for the right camera
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # Enable stream with the right camera's serial number
        serial_number_right = '242322073736'
        self.config.enable_device(serial_number_right)

        self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

        # Start the pipeline
        self.pipeline.start(self.config)

        # Create an align object
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

        # Get the camera intrinsics
        self.intrinsics = None
        self.get_intrinsics()

        self.timer = self.create_timer(1/30, self.publish_images)

    def get_intrinsics(self):
        """Fetch camera intrinsics from the RealSense device."""
        profile = self.pipeline.get_active_profile()
        stream_profile = profile.get_stream(rs.stream.color)
        intr = stream_profile.as_video_stream_profile().get_intrinsics()

        # Extract only the necessary values: fx, fy, cx, cy
        self.intrinsics = {
            'fx': intr.fx,
            'fy': intr.fy,
            'cx': intr.ppx,
            'cy': intr.ppy
        }

    def publish_images(self):
        try:
            frames = self.pipeline.wait_for_frames()

            # Align the depth frame to the color frame
            aligned_frames = self.align.process(frames)

            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()

            if not color_frame or not depth_frame:
                return

            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            color_msg = self.bridge.cv2_to_imgmsg(color_image, 'bgr8')
            depth_msg = self.bridge.cv2_to_imgmsg(depth_image, '16UC1')

            self.publisher_color.publish(color_msg)
            self.publisher_depth.publish(depth_msg)

            # Publish the camera intrinsics as Int32MultiArray
            self.publish_intrinsics()

        except Exception as e:
            self.get_logger().error(f'Error capturing frames: {e}')
            return

    def publish_intrinsics(self):
        if self.intrinsics is None:
            return

        # Create and publish Int32MultiArray with fx, fy, cx, cy
        intrinsics_array = Int32MultiArray()
        intrinsics_array.data = [
            int(self.intrinsics['fx']),
            int(self.intrinsics['fy']),
            int(self.intrinsics['cx']),
            int(self.intrinsics['cy'])
        ]
        self.publisher_intrinsics.publish(intrinsics_array)

    def shutdown_callback(self):
        # Stop the pipeline when the node shuts down
        self.get_logger().info('Shutting down RealSense pipeline...')
        self.pipeline.stop()

    def destroy_node(self):
        # Ensure the pipeline is stopped before destroying the node
        self.get_logger().info('Stopping the RealSense pipeline...')
        self.pipeline.stop()
        super().destroy_node()  # Call the parent class method


def main(args=None):
    rclpy.init(args=args)
    node = RearRightRealSensePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
