import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import os

class TestImagePublisher(Node):
    def __init__(self):
        super().__init__('test_image_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()
        self.declare_parameter('image_path', os.path.expanduser('~/.gazebo/models/igvc_track/materials/textures/track_floor.png'))
        self.image_path = self.get_parameter('image_path').get_parameter_value().string_value
        # Ensure the path is expanded if it contains a tilde
        if self.image_path.startswith('~'):
            self.image_path = os.path.expanduser(self.image_path)
        
        # Load the image once
        self.img = None
        if os.path.exists(self.image_path):
            self.img = cv2.imread(self.image_path)
            if self.img is None:
                self.get_logger().error(f'Failed to load image: {self.image_path}')
        else:
            self.get_logger().warn(f'Image path does not exist: {self.image_path}')
        
        # Set timer to publish at 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        if self.img is not None:
            msg = self.bridge.cv2_to_imgmsg(self.img, encoding='bgr8')
            self.publisher_.publish(msg)
            self.get_logger().debug(f'Published test image from {self.image_path}')
        

def main(args=None):
    rclpy.init(args=args)
    node = TestImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
