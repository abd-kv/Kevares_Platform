import base64
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import pika
import cv2
from cv_bridge import CvBridge

class CameraToRabbitMQNode(Node):

    def __init__(self):
        super().__init__('camera_to_rabbitmq_node')

        # RabbitMQ connection setup
        self.rabbitmq_connection = self.connect_to_rabbitmq()
        self.channel = self.rabbitmq_connection.channel()
        self.channel.queue_declare(queue='camera_feed')

        # Initialize CV Bridge
        self.bridge = CvBridge()

        # Subscribe to the camera topic
        self.create_subscription(Image, 'color_image', self.camera_callback, 10)

        self.get_logger().info("Camera to RabbitMQ node has been started.")

    def connect_to_rabbitmq(self):
        credentials = pika.PlainCredentials('user', 'password')
        connection = pika.BlockingConnection(pika.ConnectionParameters(host='15.222.70.94', credentials=credentials))
        self.get_logger().info("Connected to RabbitMQ.")
        return connection

    # def camera_callback(self, msg):
    #     # Convert ROS Image message to OpenCV image
    #     cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    #     # Resize the image to 480p resolution (854x480)
    #     resized_image = cv2.resize(cv_image, (450, 240))
        
    #     encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 10]  # Quality set to 30
    #     _, buffer = cv2.imencode('.jpg', resized_image, encode_param)

    #     #img_bytes = buffer.tobytes()
        
    #     # Optionally, encode the image to base64 if needed
    #     image_base64 = base64.b64encode(buffer).decode('utf-8')
        
    #     # Publish the image to RabbitMQ
    #     try:
    #         self.channel.basic_publish(
    #             exchange='',
    #             routing_key='camera_feed',  # Ensure this matches the queue name
    #             body=image_base64
    #         )
    #         self.get_logger().info("Camera image sent to RabbitMQ.")
    #     except pika.exceptions.AMQPError as error:
    #         self.get_logger().error(f"Error sending camera image: {error}")

    # def destroy_node(self):
    #     if self.rabbitmq_connection:
    #         self.rabbitmq_connection.close()
    #         self.get_logger().info("RabbitMQ connection closed.")
    #     super().destroy_node()

def camera_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert to grayscale
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Resize the image to 480p resolution (854x480)
        resized_image = cv2.resize(gray_image, (450, 240))

        # Apply DCT
        dct_image = cv2.dct(np.float32(resized_image))

        # Quantize the DCT coefficients
        # Simplified quantization matrix
        quantization_matrix = np.ones(dct_image.shape) * 10
        quantized_dct = np.round(dct_image / quantization_matrix)

        # Encode the quantized coefficients
        encoded_dct = quantized_dct.astype(np.int16).tobytes()

        # Optionally, encode the data to base64 if needed
        image_base64 = base64.b64encode(encoded_dct).decode('utf-8')

        # Publish the encoded data to RabbitMQ
        try:
            self.rabbitmq_channel.basic_publish(
                exchange='',
                routing_key='camera_feed',
                body=image_base64
            )
            self.get_logger().info("Camera image sent to RabbitMQ.")
        except pika.exceptions.AMQPError as error:
            self.get_logger().error(f"Error sending camera image: {error}")

def main(args=None):
    rclpy.init(args=args)

    node = CameraToRabbitMQNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Camera to RabbitMQ node interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
