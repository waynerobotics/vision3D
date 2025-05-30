import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import signal
import sys

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        
        self.declare_parameter('video_source', 3)
        self.declare_parameter('image_topic', 'camera/image_raw')
        self.declare_parameter('output_width', 1280)
        self.declare_parameter('output_height', 800) 

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value                
        video_source = self.get_parameter('video_source').get_parameter_value().integer_value
        self.output_width = self.get_parameter('output_width').get_parameter_value().integer_value
        self.output_height = self.get_parameter('output_height').get_parameter_value().integer_value

        self.publisher_ = self.create_publisher(Image, image_topic, 10)
        self.timer = self.create_timer(0.01, self.timer_callback)  # Increase frequency
        self.cap = cv2.VideoCapture(video_source)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2880)  # Set width
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1440)  # Set height
        self.cap.set(cv2.CAP_PROP_FPS, 30)  # Set FPS
        self.bridge = CvBridge()

        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open video source: {video_source}")
            rclpy.shutdown()
            sys.exit(1)
    
    def cleanup(self):
        """Release camera resources properly"""
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
            self.get_logger().info('Camera resources released')
    
    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret and frame is not None:
            # height, width, _ = frame.shape
            # crop_top = frame[int(height * 0.3):height, 0:width]  # Crop the top 30%
            # reduced_img = cv2.resize(crop_top, (self.output_width, self.output_height))  # Resize to desired dimensions
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)
        else:
            self.get_logger().error('Failed to capture image or empty frame received')

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    
    # Setup signal handler for graceful shutdown
    def signal_handler(sig, frame):
        image_publisher.get_logger().info('Interrupt received, shutting down...')
        image_publisher.cleanup()
        image_publisher.destroy_node()
        rclpy.shutdown()
        sys.exit(0)
    
    # Register signal handler
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        rclpy.spin(image_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure cleanup happens even if there's another exception
        image_publisher.cleanup()
        image_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()