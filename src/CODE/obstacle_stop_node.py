import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray, Float32
from ultralytics import YOLO
from cv_bridge import CvBridge
import numpy as np
import cv2

class ObstacleStopNode(Node):

    def __init__(self):
        super().__init__('obstacle_stop_node')

        self.subscription_color = self.create_subscription(Image, 'color_image', self.color_callback, 10)
        self.subscription_depth = self.create_subscription(Image, 'depth_image', self.depth_callback, 10)

        self.speed_factor_pub = self.create_publisher(Float32, '/obstacle_speed_factor', 10)
        # self.publisher = self.create_publisher(Int32MultiArray, 'movement_commands', 10)

        self.bridge = CvBridge()
        self.model = YOLO('/home/user/yolo12n.pt')  # ✅ Load your YOLO model here
        self.depth_frame = None

        self.target_class_ids = {0, 32, 34, 35, 39, 47}  # Adjust as needed (e.g., person, car, etc.)
        self.stop_distance = 2.0  # Meters

    def color_callback(self, msg):
        color_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(color_image)

        stop_robot = False

        if results and len(results[0].boxes) > 0:
            for result in results:
                for bbox, cls in zip(result.boxes.xyxy, result.boxes.cls):
                    if int(cls) in self.target_class_ids:
                        x1, y1, x2, y2 = map(int, bbox)
                        cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

                        if self.depth_frame is not None:
                            distance = self.get_depth_at_center(self.depth_frame, (x1, y1, x2, y2))
                            label = f'Class: {int(cls)}, Distance: {distance:.2f} m'
                            cv2.putText(color_image, label, (x1, y1 - 10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                            if distance <= self.stop_distance:
                                stop_robot = True
                                self.get_logger().info(f"🛑 Obstacle within {distance:.2f}m → STOPPING")
                                break
                if stop_robot:
                    break

        factor = 0.0 if stop_robot else 1.0
        self.speed_factor_pub.publish(Float32(data=factor))

        cv2.imshow('Detection', color_image)
        cv2.waitKey(1)

    def depth_callback(self, msg):
        self.depth_frame = self.bridge.imgmsg_to_cv2(msg, '16UC1')

    def get_depth_at_center(self, depth_frame, bbox):
        if depth_frame is None:
            return float('inf')
        region = depth_frame[bbox[1]:bbox[3], bbox[0]:bbox[2]]
        valid = region[region > 0]
        if valid.size == 0:
            return float('inf')
        return np.median(valid) * 0.001  # mm → m

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleStopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
