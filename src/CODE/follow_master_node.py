import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from ultralytics import YOLO
from cv_bridge import CvBridge
import numpy as np
import cv2
import time

class FollowMasterNode(Node):

    def __init__(self):
        super().__init__('follow_master_node')
        self.subscription_color = self.create_subscription(Image, 'color_image', self.color_callback, 10)
        self.subscription_depth = self.create_subscription(Image, 'depth_image', self.depth_callback, 10)
        self.subscription_dodging = self.create_subscription(Int32MultiArray, 'dodging_override', self.dodging_callback, 10)
        self.publisher = self.create_publisher(Int32MultiArray, 'movement_commands', 10)
        self.bridge = CvBridge()
        self.model = YOLO('/home/user/ros2_ws/src/abd_package/python_nodes/best.pt')
        self.depth_frame = None
        self.target_class_id = 0  # Assuming 'QR with vest' has class ID 0
        self.dodging_override = None

    def color_callback(self, msg):
        color_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(color_image)

        if len(results) == 0 or len(results[0].boxes) == 0:
            self.calculate_and_publish_movement(0, [0, 0, 1280])
        else:
            for result in results:
                for bbox, cls in zip(result.boxes.xyxy, result.boxes.cls):
                    if int(cls) == self.target_class_id:
                        x1, y1, x2, y2 = map(int, bbox)
                        cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

                        distance = self.get_depth_at_center(self.depth_frame, (x1, y1, x2, y2))
                        label = f'Distance: {distance:.2f} meters'
                        cv2.putText(color_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                        self.calculate_and_publish_movement(distance, [x1, y1, x2, y2])

        #cv2.imshow('Detection', color_image)
        #cv2.waitKey(1)

    def depth_callback(self, msg):
        self.depth_frame = self.bridge.imgmsg_to_cv2(msg, '16UC1')

    def get_depth_at_center(self, depth_frame, bbox):
        if depth_frame is None:
            return 0
        depth_values = depth_frame[bbox[1]:bbox[3], bbox[0]:bbox[2]].flatten()
        distance = np.median(depth_values) * 0.001  # Convert to meters
        return distance

    def dodging_callback(self, msg):
        self.dodging_override = None #msg.data  # Store the dodging override

    def calculate_and_publish_movement(self, distance, bbox):
        stop = 0
        if self.dodging_override is not None:
            steer = 0
            speed = 0
            # stop = self.dodging_override[2]
            self.dodging_override = None  # Reset after using it
            time.sleep(0.3)
            
            
        else:
            x_center = (bbox[0] + bbox[2]) / 2
            deflection = x_center - 640

            steer = -576 * deflection / 320
            steer = int(steer)

            # stop = 0

            speed = (distance - 1.6) * 1000
            speed = max(0, min(1500, int(speed)))
        # if stop == 1:
        #     speed = 0
        msg = Int32MultiArray()
        msg.data = [speed, steer]
        self.publisher.publish(msg)
        # self.get_logger().info(f'Published: speed {speed} mm/s, steer {steer}')

def main(args=None):
    rclpy.init(args=args)
    node = FollowMasterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

