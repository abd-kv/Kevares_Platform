import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String
from cv_bridge import CvBridge
import numpy as np
import serial
import serial.tools.list_ports


class DodgingNode(Node):
    def __init__(self):
        super().__init__('dodging_node')

        # === Depth topics from all RealSense cameras ===
        self.camera_topics = [
            'depth_image',
            'top_left_depth_image',
            'top_right_depth_image',
            'trailer_rear_depth_image'
        ]

        # === Camera groups ===
        self.front_group = {'depth_image', 'top_left_depth_image', 'top_right_depth_image'}
        self.rear_group = {'trailer_rear_depth_image'}

        # === Arduino serials ===
        self.arduino_serial_front = "55834323833351A07090"   # Arduino #1
        self.arduino_serial_rear = "34330313531351719100"    # Arduino #2

        self.bridge = CvBridge()
        self.depth_frames = {}
        self.swd_enabled = False

        self.current_led_state_front = None
        self.current_led_state_rear = None

        # Subscribe to all cameras
        for topic in self.camera_topics:
            self.create_subscription(Image, topic,
                                     lambda msg, t=topic: self.depth_callback(msg, t),
                                     10)
            self.get_logger().info(f"Subscribed to {topic}")

        # Publishers
        self.publisher_factor = self.create_publisher(Float32, '/obstacle_speed_factor', 10)
        self.publisher_alert = self.create_publisher(String, '/obstacle_alert', 10)
        self.subscription_can = self.create_subscription(String, 'can_feedback', self.listener_callback_can, 10)

        # === Arduino Serial Setup ===
        self.serial_front = self.connect_arduino(self.arduino_serial_front)
        self.serial_rear = self.connect_arduino(self.arduino_serial_rear)

        # Parameters
        self.low = 0.8   # meters
        self.high = 1.5  # meters

    # --------------------- Find Arduino by serial number ---------------------
    def find_arduino_port(self, serial_number):
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if port.serial_number == serial_number:
                return port.device
        return None

    # --------------------- Connect to Arduino ---------------------
    def connect_arduino(self, serial_number):
        port = self.find_arduino_port(serial_number)
        if not port:
            self.get_logger().warn(f"Arduino {serial_number} not found.")
            return None
        try:
            ser = serial.Serial(port, 9600)
            self.get_logger().info(f"Connected to Arduino {serial_number} on {port}")
            return ser
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Arduino {serial_number} ({port}): {e}")
            return None

    # --------------------- Depth callback ---------------------
    def depth_callback(self, msg, topic_name):
        if self.swd_enabled:
            return

        try:
            depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"[{topic_name}] Failed to convert depth image: {e}")
            return

        self.depth_frames[topic_name] = depth_frame

        # Evaluate front or rear group
        if topic_name in self.front_group:
            self.evaluate_group_depths(self.front_group, "front")
        elif topic_name in self.rear_group:
            self.evaluate_group_depths(self.rear_group, "rear")

    # --------------------- Evaluate depths per group ---------------------
    def evaluate_group_depths(self, group, label):
        min_dists = []

        for name in group:
            frame = self.depth_frames.get(name)
            if frame is None:
                continue

            if frame.dtype == np.uint16:
                depth_image = frame.astype(np.float32) / 1000.0
            else:
                depth_image = frame

            h, w = depth_image.shape
            region = depth_image[int(h * 0.6):int(h * 0.65), :]
            valid = region[(region > 0.2) & (region < 2.5) & np.isfinite(region)]

            if valid.size > 0:
                min_dists.append(float(np.min(valid)))

        if not min_dists:
            self.get_logger().warn(f"[{label.upper()}] No valid depth points found.")
            self.set_led(label, 'OFF')
            return

        closest = min(min_dists)
        self.get_logger().info(f"[{label.upper()}] Closest obstacle: {closest:.2f} m")

        # Decide LED command
        if closest > self.high:
            cmd = 'OFF'
        elif self.low < closest <= self.high:
            cmd = 'BLINK'
        else:
            cmd = 'SOLID'

        self.set_led(label, cmd)

    # --------------------- Send LED Command ---------------------
    def set_led(self, group_label, command):
        if group_label == "front":
            if command == self.current_led_state_front:
                return
            ser = self.serial_front
            self.current_led_state_front = command
        elif group_label == "rear":
            if command == self.current_led_state_rear:
                return
            ser = self.serial_rear
            self.current_led_state_rear = command
        else:
            return

        # 🔸 Send the same serial message format as before (simple command only)
        if ser and ser.is_open:
            try:
                ser.write(f"{command}\n".encode())
                self.get_logger().info(f"[{group_label.upper()}] Sent: {command}")
            except Exception as e:
                self.get_logger().error(f"[{group_label.upper()}] Serial send failed: {e}")
        else:
            self.get_logger().warn(f"[{group_label.upper()}] Serial not connected.")

        # Keep alert topic informative (still shows which side)
        alert_msg = String()
        alert_msg.data = f"{group_label}:{command}"
        self.publisher_alert.publish(alert_msg)

    # --------------------- CAN Feedback ---------------------
    def listener_callback_can(self, msg):
        try:
            data = json.loads(msg.data)
            if data.get("id") == "0x241":
                swd = data.get("swd")
                if swd == 1:
                    self.swd_enabled = True
                    self.publisher_factor.publish(Float32(data=1.0))
                    self.set_led("front", "OFF")
                    self.set_led("rear", "OFF")
                    self.get_logger().info("✅ SWD ON - Obstacle avoidance bypassed.")
                else:
                    self.swd_enabled = False
                    self.get_logger().info("❎ SWD OFF - Obstacle avoidance active.")
        except Exception as e:
            self.get_logger().error(f"Error processing CAN data: {e}")


# --------------------- MAIN ---------------------
def main(args=None):
    rclpy.init(args=args)
    node = DodgingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()