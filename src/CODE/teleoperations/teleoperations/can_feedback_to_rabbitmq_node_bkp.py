import rclpy
from rclpy.node import Node
import can
import json
from std_msgs.msg import String
import pika


class CanFeedbackToRabbitMQNode(Node):
    """
    CanFeedbackToRabbitMQNode is a ROS2 node that interfaces with a CAN bus to receive and process CAN messages,
    and publishes the processed data to a RabbitMQ queue.

    Attributes:
        can_interface (str): The CAN interface to use (default is 'can0').
        bus (can.interface.Bus): The CAN bus interface.
        publisher_ (Publisher): The ROS2 publisher for the 'can_feedback' topic.
        timer (Timer): The ROS2 timer for periodic callbacks.
        rabbitmq_connection (pika.BlockingConnection): The RabbitMQ connection.
        rabbitmq_channel (pika.channel.Channel): The RabbitMQ channel.
        rabbitmq_queue (str): The RabbitMQ queue name.
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
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.get_logger().info("CAN Feedback to RabbitMQ node started")

        # Initialize RabbitMQ connection
        self.rabbitmq_connection = self.connect_to_rabbitmq()
        self.rabbitmq_channel = self.rabbitmq_connection.channel()
        self.rabbitmq_queue = 'telemetry_data'
        self.rabbitmq_channel.queue_declare(queue=self.rabbitmq_queue)

    def connect_to_rabbitmq(self):
        try:
            credentials = pika.PlainCredentials('user', 'password')
            connection = pika.BlockingConnection(pika.ConnectionParameters(
                host='15.222.70.94', credentials=credentials))
            self.get_logger().info("Connected to RabbitMQ.")
            return connection
        except pika.exceptions.AMQPConnectionError as e:
            self.get_logger().error(f"Failed to connect to RabbitMQ: {e}")
            raise RuntimeError("Failed to connect to RabbitMQ")

    def timer_callback(self):
        try:
            while True:
                message = self.bus.recv()  # Timeout of 0.01 seconds to allow periodic checking
                self.process_message(message)
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

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
        output = {}

        if (message.arbitration_id) == int('0x221', 16):
            moving_speed_raw = int.from_bytes(
                data[0:2], byteorder='big', signed=True)
            # Convert to m/s with accuracy of 0.001 m/s
            moving_speed = moving_speed_raw * 0.001
            corner_angle_raw = int.from_bytes(
                data[6:8], byteorder='big', signed=True)
            # Convert to rad with accuracy of 0.001 rad
            corner_angle = corner_angle_raw * 0.001
            output.update({
                "id": "0x221",
                "moving_speed": moving_speed,
                "corner_angle": corner_angle
            })

        if (message.arbitration_id) == int('0x211', 16):
            current_status = data[0]
            mode_control = data[1]
            battery_voltage_raw = int.from_bytes(
                data[2:4], byteorder='big', signed=False)
            # Convert to volts with accuracy of 0.1V
            battery_voltage = battery_voltage_raw * 0.1
            failure_info_high = data[4]
            failure_info_low = data[5]
            parking_state = data[6]
            output.update({
                "id": "0x211",
                "current_status": current_status,
                "mode_control": mode_control,
                "battery_voltage": battery_voltage,
                "failure_info_high": failure_info_high,
                "failure_info_low": failure_info_low,
                "parking_state": parking_state
            })

        if (message.arbitration_id) == int('0x251', 16):
            motor_speed = int.from_bytes(
                data[0:2], byteorder='big', signed=True)
            motor_current = int.from_bytes(
                data[2:4], byteorder='big', signed=True)
            motor_position = int.from_bytes(
                data[4:8], byteorder='big', signed=True)
            motor_speed_corrected = motor_speed  # assuming the offset is +1500
            output.update({
                "id": "0x251",
                "motor_speed": motor_speed_corrected,
                "motor_current": motor_current,
                "motor_position": motor_position
            })

        if (message.arbitration_id) == int('0x361', 16):
            battery_soc = data[0]  # Battery SOC (State of Charge)
            battery_soh = data[1]  # Battery SOH (State of Health)
            output.update({
                "id": "0x361",
                "battery_soc": battery_soc,
                "battery_soh": battery_soh
            })

        if output:
            self.publish_data(output)


    def publish_data(self, data):
        json_data = json.dumps(data)
        self.publisher_.publish(String(data=json_data))
        self.rabbitmq_channel.basic_publish(
            exchange='',
            routing_key=self.rabbitmq_queue,
            body=json_data
        )
        self.get_logger().info(f"Published CAN data to RabbitMQ: {json_data}")

    def destroy_node(self):
        super().destroy_node()
        self.rabbitmq_connection.close()
        self.get_logger().info("RabbitMQ connection closed")


def main(args=None):
    rclpy.init(args=args)
    node = CanFeedbackToRabbitMQNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("CAN Feedback to RabbitMQ node interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

