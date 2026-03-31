import base64
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import can
import json
import pika
from cv_bridge import CvBridge
import time
# import ffmpeg
import os  # Import the os module to read environment variables
# from dotenv import load_dotenv  # Import the load_dotenv function

# Load environment variables from .env file
# load_dotenv()


class CanFeedbackToRabbitMQNode(Node):
    """
    CanFeedbackToRabbitMQNode is a ROS2 node that interfaces with a CAN bus to receive and process CAN messages,
    captures images from a camera topic, processes them, and publishes the processed data to a RabbitMQ queue.

    Attributes:
        can_interface (str): The CAN interface to use (default is 'can0').
        bus (can.interface.Bus): The CAN bus.
        publisher_ (Publisher): The ROS2 publisher for the 'can_feedback' topic.
        timer (Timer): The ROS2 timer for periodic callbacks.
        rabbitmq_connection (pika.BlockingConnection): The RabbitMQ connection.
        rabbitmq_channel (pika.channel.Channel): The RabbitMQ channel.
        rabbitmq_queue (str): The RabbitMQ queue name.
        gps_data (dict): The latest GPS data.
        bridge (CvBridge): The CV Bridge for converting ROS Image messages to OpenCV images.
    """

    def __init__(self):
        super().__init__('can_feedback_to_rabbitmq_node')

        # Initialize CAN interface
        self.can_interface = 'can0'  # Change this to your actual CAN interface
        try:
            self.bus = can.interface.Bus(
                self.can_interface, bustype='socketcan', bitrate=500000)
        except OSError as e:
            self.get_logger().error(
                f"CAN interface {self.can_interface} initialization failed: {e}")
            raise RuntimeError(
                f"CAN interface {self.can_interface} initialization failed")

        self.publisher_ = self.create_publisher(String, 'can_feedback', 10)
        self.timer = self.create_timer(0.001, self.timer_callback)
        self.get_logger().info("can_feedback_to_rabbitmq_node node started")

        # Initialize RabbitMQ connection
        self.rabbitmq_connection = self.connect_to_rabbitmq()
        self.rabbitmq_channel = self.rabbitmq_connection.channel()
        self.rabbitmq_queue = 'telemetry_data'
        self.rabbitmq_channel.queue_declare(queue=self.rabbitmq_queue)

        # Initialize GPS data
        self.gps_data = {"lat": None, "lng": None}

        # Subscribe to GPS data topic
        self.create_subscription(String, 'gps_data', self.gps_callback, 10)

        # Initialize CV Bridge
        self.bridge = CvBridge()

        self.latest_image_msg = None
        # Subscribe to the camera topic
        self.create_subscription(Image, 'color_image',
                                 self.camera_callback, 10)
        
        self.create_timer(0.1, self.process_callback)

        # Initialize data storage
        self.data_storage = {}

        

    def connect_to_rabbitmq(self):
        try:
            # Read credentials from environment variables
            # rabbitmq_user = os.getenv('RABBITMQ_USER')
            rabbitmq_user = 'user'
            # rabbitmq_password = os.getenv('RABBITMQ_PASSWORD')
            rabbitmq_password = 'password'
            # Default to localhost if not set   
            # rabbitmq_host = os.getenv('RABBITMQ_HOST', 'localhost')
            rabbitmq_host = '15.222.70.94'

            credentials = pika.PlainCredentials(
                rabbitmq_user, rabbitmq_password)
            connection = pika.BlockingConnection(pika.ConnectionParameters(
                host=rabbitmq_host, credentials=credentials))
            self.get_logger().info("Connected to RabbitMQ.")
            return connection
        except pika.exceptions.AMQPConnectionError as e:
            self.get_logger().error(f"Failed to connect to RabbitMQ: {e}")
            raise RuntimeError("Failed to connect to RabbitMQ")

    def timer_callback(self):
        try:
            # Non-blocking receive with timeout
            message = self.bus.recv()
            if message:
                self.process_message(message)
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    def gps_callback(self, msg):
        self.get_logger().info(
            f"GPS callback triggered with message: {msg.data}")
        gps_data = json.loads(msg.data)
        self.gps_data["lat"] = gps_data.get("latitude")
        self.gps_data["lng"] = gps_data.get("longitude")
        self.get_logger().info(f"Received GPS data: {self.gps_data}")

    def camera_callback(self, msg):
        self.latest_image_msg = msg

    def process_callback(self):
        if self.latest_image_msg is not None:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image_msg, desired_encoding='bgr8')

            # Resize the image to 480p resolution (854x480)
            resized_image = cv2.resize(cv_image, (450, 240))

            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 10]  # Quality set to 10
            _, buffer = cv2.imencode('.jpg', resized_image, encode_param)

            # Optionally, encode the image to base64 if needed
            image_base64 = base64.b64encode(buffer).decode('utf-8')

            # Publish the image to RabbitMQ
            try:
                self.rabbitmq_channel.basic_publish(
                    exchange='',
                    routing_key='camera_feed',  # Ensure this matches the queue name
                    body=image_base64
                )
                self.get_logger().info("Camera image sent to RabbitMQ.")
            except pika.exceptions.AMQPError as error:
                self.get_logger().error(f"Error sending camera image: {error}")
        else:
            self.get_logger().warning("No image received.")


    def process_message(self, message):
        """
        Processes incoming CAN messages and publishes the extracted data.
        Args:
            message (object): The CAN message object containing the data and arbitration ID.
        The function handles three specific arbitration IDs:
        - 0x221: Extracts moving speed and corner angle from the message data.
        - 0x211: Extracts current status, mode control, battery voltage, failure info, and parking state from the message data.
        - 0x251: Extracts motor speed, motor current, and motor position from the message data.
        - 0x361: Extracts battery state of charge (SOC) and state of health (SOH) from the message data.
        The extracted data is then published using the `publish_data` method.
        Note:
            - Moving speed is converted to meters per second (m/s) with an accuracy of 0.001 m/s.
            - Corner angle is converted to radians (rad) with an accuracy of 0.001 rad.
            - Battery voltage is converted to volts (V) with an accuracy of 0.1V.
            - Motor speed is assumed to have an offset correction (e.g., +1500).
        Raises:
            ValueError: If the message arbitration ID is not recognized.
        """
        data = message.data
        arbitration_id = message.arbitration_id

        self.get_logger().info(
            f"Received CAN message with ID: {hex(arbitration_id)}")

        if arbitration_id == int('0x221', 16):
            moving_speed_raw = int.from_bytes(
                data[0:2], byteorder='big', signed=True)
            moving_speed = moving_speed_raw * 0.001 * 3.6 # Convert to km/h
            corner_angle_raw = int.from_bytes(
                data[6:8], byteorder='big', signed=True)
            corner_angle = corner_angle_raw * 0.001
            self.data_storage["0x221"] = {
                "moving_speed": moving_speed,
                "corner_angle": corner_angle
            }

        elif arbitration_id == int('0x211', 16):
            current_status = data[0]
            mode_control = data[1]
            battery_voltage_raw = int.from_bytes(
                data[2:4], byteorder='big', signed=False)
            battery_voltage = battery_voltage_raw * 0.1
            failure_info_high = data[4]
            failure_info_low = data[5]
            parking_state = data[6]
            self.data_storage["0x211"] = {
                "current_status": current_status,
                "mode_control": mode_control,
                "battery_voltage": battery_voltage,
                "failure_info_high": failure_info_high,
                "failure_info_low": failure_info_low,
                "parking_state": parking_state
            }

        elif arbitration_id == int('0x251', 16):
            motor_speed = int.from_bytes(
                data[0:2], byteorder='big', signed=True)
            motor_current = int.from_bytes(
                data[2:4], byteorder='big', signed=True)
            motor_position = int.from_bytes(
                data[4:8], byteorder='big', signed=True)
            self.data_storage["0x251"] = {
                "motor_speed": motor_speed,
                "motor_current": motor_current,
                "motor_position": motor_position
            }

        elif arbitration_id == int('0x361', 16):
            battery_soc = data[0]
            battery_soh = data[1]
            self.data_storage["0x361"] = {
                "battery_soc": battery_soc,
                "battery_soh": battery_soh
            }

        # Add more arbitration IDs as needed
        # elif arbitration_id == int('0xXXX', 16):
        #     ...

        # Check if we have data from all relevant IDs
        if all(key in self.data_storage for key in ["0x221", "0x211", "0x251"]):
            self.publish_data(self.data_storage)
            self.data_storage.clear()

    def publish_data(self, data):
        # Include GPS data if available
        self.get_logger().info(f"Current GPS data: {self.gps_data}")
        if self.gps_data["lat"] is not None and self.gps_data["lng"] is not None:
            data.update({
                "location": {
                    "lat": self.gps_data["lat"],
                    "lng": self.gps_data["lng"]
                }
            })
            self.get_logger().info(
                f"Including GPS data in published message: {self.gps_data}")

        json_data = json.dumps(data)
        self.publisher_.publish(String(data=json_data))
        self.rabbitmq_channel.basic_publish(
            exchange='',
            routing_key=self.rabbitmq_queue,
            body=json_data
        )
        self.get_logger().info(f"Published CAN data to RabbitMQ: {json_data}")

    def destroy_node(self):
        if self.rabbitmq_connection:
            self.rabbitmq_connection.close()
            self.get_logger().info("RabbitMQ connection closed.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CanFeedbackToRabbitMQNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Merged node interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
