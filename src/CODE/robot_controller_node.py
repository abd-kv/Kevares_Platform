import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import can
import struct

class CanCommandSubscriber(Node):

    def __init__(self):
        super().__init__('robot_controller')
        self.subscription = self.create_subscription(
            Int32MultiArray,
            'movement_commands',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Initialize CAN bus
        self.bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=500000)
        self.activate_can_mode()
        self.get_logger().info('CAN mode activated')

    def activate_can_mode(self):
        message = can.Message(arbitration_id=0x421, data=[0x01], is_extended_id=False)
        self.bus.send(message)
        self.get_logger().info('Sent CAN mode activation command')
        

    def listener_callback(self, msg):
        linear_speed, steer = msg.data
        self.send_movement_command(self.bus, linear_speed, steer)

    def send_movement_command(self, bus, linear_speed, steer):
        data = struct.pack('>h4xh', linear_speed, steer)  # '4x' adds four zero bytes
        message = can.Message(arbitration_id=0x111, data=data, is_extended_id=False)
        bus.send(message, timeout=0.01)
        self.get_logger().info(f'Sent linear {linear_speed} mm/s, steer {steer}')

    def destroy_node(self):
        self.get_logger().info('Shutting down CAN bus interface')
        self.bus.shutdown()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CanCommandSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
