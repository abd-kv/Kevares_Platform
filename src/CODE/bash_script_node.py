import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import json
import os
import signal

class CanFeedbackControlNode(Node):
    def __init__(self):
        super().__init__('can_feedback_control_node')
        self.subscription = self.create_subscription(
            String,
            'can_feedback',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.process = None
        self.script_path = "/home/user/start_zigzag.sh"  # change this

        self.get_logger().info("CAN Feedback Control Node initialized.")

    def listener_callback(self, msg):
        try:
            data = json.loads(msg.data)

            if data.get("id") != "0x241":
                return

            swc = data.get("swc")
            if swc == 2:
                if self.process is None or self.process.poll() is not None:
                    self.get_logger().info("Starting script...")
                    self.process = subprocess.Popen(
                        ['bash', self.script_path],
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE,
                        start_new_session=True  # creates a new process group
                    )
                else:
                    self.get_logger().info("Script already running.")
            elif swc == 3:
                if self.process and self.process.poll() is None:
                    self.get_logger().info("Stopping script and all its children...")
                    os.killpg(self.process.pid, signal.SIGTERM)
                    self.process = None
                else:
                    self.get_logger().info("No running script to stop.")
        except Exception as e:
            self.get_logger().error(f"Error processing message: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = CanFeedbackControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down due to user interruption.")
    finally:
        if node.process and node.process.poll() is None:
            os.killpg(os.getpgid(node.process.pid), signal.SIGTERM)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
