import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import os

class TestImagePublisher(Node):
    def __init__(self):
        super().__init__('test_image_publisher')
        self.publisher_ = self.create_publisher(Image, '/image_raw', 10)
        self.bridge = CvBridge()
        self.declare_parameter('image_path', '/tmp/test_image.png')  # Default path
        self.image_path = self.get_parameter('image_path').get_parameter_value().string_value
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.published = False

    def timer_callback(self):
        if not self.published:
            if os.path.exists(self.image_path):
                img = cv2.imread(self.image_path)
                if img is not None:
                    msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
                    self.publisher_.publish(msg)
                    self.get_logger().info(f'Published test image from {self.image_path}')
                    self.published = True
                else:
                    self.get_logger().error(f'Failed to load image: {self.image_path}')
            else:
                self.get_logger().warn(f'Image path does not exist: {self.image_path}')


def main(args=None):
    rclpy.init(args=args)
    node = TestImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
