import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import numpy as np
import serial
import serial.tools.list_ports

class DodgingNode(Node):
    def __init__(self):
        super().__init__('dodging_node')

        # Update this if your camera uses a different topic
        self.subscription_depth = self.create_subscription(Image, 'depth_image', self.depth_callback, 10)

        self.publisher_factor = self.create_publisher(
            Float32,
            '/obstacle_speed_factor',
            10
        )

        self.bridge = CvBridge()
        self.depth_frame = None

        # === Arduino Serial Setup ===
        self.arduino_serial_number = "55834323833351A07090"  # Replace with yours
        self.arduino_port = self.find_arduino_port(self.arduino_serial_number)
        self.ser = None
        if self.arduino_port:
            try:
                self.ser = serial.Serial(self.arduino_port, 9600, timeout=1)
                self.get_logger().info(f"Connected to Arduino on port {self.arduino_port}")
            except serial.SerialException as e:
                self.get_logger().error(f"Serial connection failed: {e}")
        else:
            self.get_logger().warn("⚠️ Arduino not found. Continuing without LED control.")

        self.get_logger().info("DodgingNode initialized (Real Camera + Arduino)")

    def find_arduino_port(self, serial_number):
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if port.serial_number == serial_number:
                return port.device
        return None

    def depth_callback(self, msg):
        try:
            # RealSense may use 16UC1 → convert to meters
            self.depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Failed to convert depth image: {e}")
            return
        self.process_depth_image()

    def process_depth_image(self):
        if self.depth_frame is None:
            return

        # Convert 16UC1 to meters if needed
        if self.depth_frame.dtype == np.uint16:
            depth_image = self.depth_frame.astype(np.float32) / 1000.0
        else:
            depth_image = self.depth_frame

        height, width = depth_image.shape
        region = depth_image[:int(height * 3 / 4), :]
        valid_depths = region[(region > 0.2) & np.isfinite(region) & (region < 10.0)]

        if valid_depths.size == 0:
            factor = 1.0
            self.control_led('STEADY')
        else:
            min_dist = float(np.min(valid_depths))
            if min_dist < 1.5:
                factor = 0.0
                self.control_led('BLINK')
            elif 1.5 <= min_dist <= 2.0:
                factor = (min_dist - 1.5) / 0.5
                self.control_led('BLINK')
            else:
                factor = 1.0
                self.control_led('STEADY')

            self.get_logger().info(f"Min depth: {min_dist:.2f} m → Speed factor: {factor:.2f}")

        self.publisher_factor.publish(Float32(data=factor))

    def control_led(self, command):
        if self.ser and self.ser.is_open:
            if command == 'BLINK':
                self.ser.write(b'BLINK\n')
            elif command == 'STEADY':
                self.ser.write(b'STEADY\n')
            self.get_logger().info(f"LED command sent: {command}")

def main(args=None):
    rclpy.init(args=args)
    node = DodgingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
