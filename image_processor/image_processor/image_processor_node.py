#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageProcessorNode(Node):
    def __init__(self):
        super().__init__('image_processor')

        # Declare parameters
        self.declare_parameter('process_flag', False)

        # Initialize topics
        self.run_process_sub = self.create_subscription(String, '/run_process', self.run_process_callback, 10)
        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.processed_image_pub = self.create_publisher(Image, '/processed_image', 10)

        self.bridge = CvBridge()
        self.process_image = False  # Flag to process image

    def run_process_callback(self, msg):
        """Callback to handle /run_process messages."""
        if msg.data == 'y':
            self.get_logger().info("Received 'y': Image processing enabled.")
            self.process_image = True
        elif msg.data == 'n':
            self.get_logger().info("Received 'n': Image processing disabled.")
            self.process_image = False
        else:
            self.get_logger().warn(f"Invalid input on /run_process: {msg.data}")

    def image_callback(self, msg):
        """Callback to handle /image_raw messages."""
        if self.process_image:
            try:
                # Convert ROS Image message to OpenCV format
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

                # Convert to grayscale
                gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
                inverted_image = cv2.bitwise_not(gray_image)

                # Resize to 10x10
                resized_image = cv2.resize(inverted_image, (10, 10), interpolation=cv2.INTER_AREA)

                # Normalize and cast to float32
                normalized_image = (resized_image / 255.0).astype('float32')

                # Convert back to ROS Image message and publish
                ros_image = self.bridge.cv2_to_imgmsg(normalized_image, encoding="32FC1")
                self.processed_image_pub.publish(ros_image)

                self.get_logger().info("Processed and published the image.")
            except Exception as e:
                self.get_logger().error(f"Error processing image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
