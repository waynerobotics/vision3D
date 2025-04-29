# 
#  This node is currently not used.  #################################
# 
#   I pulled it in if we want to convert the fusion image to a laser scan.
#         This would need to be reworked to take a point cloud as input and convert it to a laser scan.
# 

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageToLaserScan(Node):
    def __init__(self):
        super().__init__('image_to_laser_scan')
        self.subscription = self.create_subscription(
            Image,
            'lane_detection/segemntation_mask',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(LaserScan, 'camera/laser_scan', 10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            # Convert the image to grayscale
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Find the non-zero points in the grayscale image
            points = cv2.findNonZero(gray_image)

            if points is not None:
                # Initialize LaserScan message
                scan = LaserScan()
                scan.header.stamp = self.get_clock().now().to_msg()
                scan.header.frame_id = 'camera_frame'
                scan.angle_min = -np.pi / 2
                scan.angle_max = np.pi / 2
                scan.angle_increment = np.pi / len(points)
                scan.range_min = 0.0
                scan.range_max = 10.0
                scan.ranges = [float('inf')] * len(points)

                # Calculate the offset for the LaserScan message
                for point in points:
                    x, y = point[0]
                    index = int((x / gray_image.shape[1]) * len(points))
                    distance = gray_image.shape[0] - y
                    scan.ranges[index] = distance

                # Publish the LaserScan message
                self.publisher_.publish(scan)
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

def main(args=None):
    rclpy.init(args=args)
    image_to_laser_scan = ImageToLaserScan()
    rclpy.spin(image_to_laser_scan)
    image_to_laser_scan.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()