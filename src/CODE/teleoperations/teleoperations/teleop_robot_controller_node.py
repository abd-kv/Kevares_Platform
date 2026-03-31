import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import pika
import json
import threading
from python_nodes.follow_master_node import FollowMasterNode  # Correct import

class FollowMasterNodeWrapper:
    """
    A wrapper class for FollowMasterNode to add start_node and stop_node methods.
    """

    def __init__(self):
        self.node = FollowMasterNode()
        self.active = False
        self.thread = None

    def start_node(self):
        if not self.active:
            self.active = True
            self.node.get_logger().info("FollowMasterNode started")
            # Start a thread to run the node's main logic
            self.thread = threading.Thread(target=self.run_node)
            self.thread.start()

    def stop_node(self):
        if self.active:
            self.active = False
            self.node.get_logger().info("FollowMasterNode stopped")
            # Stop the thread running the node's main logic
            if self.thread is not None:
                self.thread.join()
                self.thread = None

    def run_node(self):
        # Main logic of the FollowMasterNode
        while self.active:
            rclpy.spin_once(self.node, timeout_sec=1.0)

    def destroy_node(self):
        self.stop_node()
        self.node.destroy_node()


class TeleOpCanCommandSubscriber(Node):
    """
    A ROS2 node that subscribes to movement commands and sends them to a robot via CAN bus.
    It also consumes commands from RabbitMQ queues.
    Methods
    -------
    __init__():
        Initializes the node, CAN bus, and RabbitMQ connections.
    connect_to_rabbitmq(queue_name):
        Establishes a connection to RabbitMQ and returns the connection object.
    rabbitmq_callback(ch, method, properties, body):
        Callback function for processing messages from the 'robot_commands' queue.
    mode_callback(ch, method, properties, body):
        Callback function for processing messages from the 'robot_modes' queue.
    listener_callback(msg):
        Callback function for processing ROS2 messages from the 'movement_commands' topic.
    send_movement_command(bus, linear_speed, steer):
        Sends a movement command to the robot via CAN bus.
    send_can_message(message, retry_count=3, retry_delay=0.1):
        Sends a CAN message with retry logic.
    destroy_node():
        Shuts down the CAN bus interface and closes the RabbitMQ connections.
    """

    def __init__(self):
        super().__init__('teleop_rabbit_robot_controller')
        self.publisher = self.create_publisher(
            Int32MultiArray, 'movement_commands', 10)

        # Initialize RabbitMQ connections
        self.rabbitmq_connection_commands = self.connect_to_rabbitmq(
            'robot_commands')
        self.rabbitmq_channel_commands = self.rabbitmq_connection_commands.channel()
        self.rabbitmq_channel_commands.queue_declare(queue='robot_commands')

        self.rabbitmq_connection_modes = self.connect_to_rabbitmq(
            'robot_modes')
        self.rabbitmq_channel_modes = self.rabbitmq_connection_modes.channel()
        self.rabbitmq_channel_modes.queue_declare(queue='robot_modes')

        # Start consuming messages from RabbitMQ
        self.rabbitmq_channel_commands.basic_consume(
            queue='robot_commands',
            on_message_callback=self.rabbitmq_callback,
            auto_ack=True
        )
        self.rabbitmq_channel_modes.basic_consume(
            queue='robot_modes',
            on_message_callback=self.mode_callback,
            auto_ack=True
        )
        self.get_logger().info("Started consuming RabbitMQ messages")
        threading.Thread(target=self.rabbitmq_channel_commands.start_consuming).start()
        threading.Thread(target=self.rabbitmq_channel_modes.start_consuming).start()

        # Initialize FollowMasterNodeWrapper
        self.follow_master_node = FollowMasterNodeWrapper()

    def connect_to_rabbitmq(self, queue_name):
        try:
            rabbitmq_user = 'user'
            rabbitmq_password = 'password'
            rabbitmq_host = '15.222.70.94'

            credentials = pika.PlainCredentials(
                rabbitmq_user, rabbitmq_password)
            connection = pika.BlockingConnection(pika.ConnectionParameters(
                host=rabbitmq_host, credentials=credentials))
            self.get_logger().info(f"Connected to RabbitMQ queue: {queue_name}.")
            return connection
        except pika.exceptions.AMQPConnectionError as e:
            self.get_logger().error(f"Failed to connect to RabbitMQ queue: {queue_name}: {e}")
            raise RuntimeError("Failed to connect to RabbitMQ")

    def rabbitmq_callback(self, ch, method, properties, body):
        try:
            command_data = json.loads(body.decode('utf-8'))
            self.get_logger().info(
                f"Received command from RabbitMQ: {command_data}")
            # Check if the command is valid and publish it
            command = command_data.get("command", [])
            if len(command) == 2:
                speed = command[0]
                steer = command[1]
                msg = Int32MultiArray()
                msg.data = [speed, steer]
                self.publisher.publish(msg)
                self.get_logger().info(
                    f'Published: speed {speed} mm/s, steer {steer}')
            else:
                self.get_logger().warning("Invalid command format received")
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to decode JSON message: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")

    def mode_callback(self, ch, method, properties, body):
        try:
            command_data = json.loads(body.decode('utf-8'))
            self.get_logger().info(
                f"Received mode command from RabbitMQ: {command_data}")
            # Check if the mode is "follow_master_node" to start or stop FollowMasterNode
            if command_data.get("mode") == "follow_master_node":
                if not self.follow_master_node.active:
                    self.get_logger().info("Starting FollowMasterNode")
                    self.follow_master_node.start_node()
                else:
                    self.get_logger().info("Stopping FollowMasterNode")
                    self.follow_master_node.stop_node()
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to decode JSON message: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")

    def destroy_node(self):
        if self.rabbitmq_connection_commands:
            self.rabbitmq_connection_commands.close()
            self.get_logger().info("RabbitMQ connection for commands closed.")
        if self.rabbitmq_connection_modes:
            self.rabbitmq_connection_modes.close()
            self.get_logger().info("RabbitMQ connection for modes closed.")
        self.follow_master_node.destroy_node()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TeleOpCanCommandSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()