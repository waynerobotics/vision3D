#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        
        # Parameters
        self.declare_parameter('roi_top_crop_percent', 30.0)
        self.declare_parameter('downsample_factor', 0.5)
        self.declare_parameter('white_threshold_lower', [0, 0, 180])
        self.declare_parameter('white_threshold_upper', [180, 50, 255])
        self.declare_parameter('canny_low_threshold', 50)
        self.declare_parameter('canny_high_threshold', 150)
        self.declare_parameter('hough_line_threshold', 50)
        self.declare_parameter('hough_min_line_length', 50)
        self.declare_parameter('hough_max_line_gap', 10)
        self.declare_parameter('morph_kernel_size', 3)
        self.declare_parameter('morph_iterations', 2)
        self.declare_parameter('min_circle_radius', 10)
        self.declare_parameter('max_circle_radius', 50)
        
        # Load parameters
        self.roi_top_crop_percent = self.get_parameter('roi_top_crop_percent').value
        self.downsample_factor = self.get_parameter('downsample_factor').value
        self.white_lower = np.array(self.get_parameter('white_threshold_lower').value)
        self.white_upper = np.array(self.get_parameter('white_threshold_upper').value)
        self.canny_low = self.get_parameter('canny_low_threshold').value
        self.canny_high = self.get_parameter('canny_high_threshold').value
        self.hough_threshold = self.get_parameter('hough_line_threshold').value
        self.hough_min_line_length = self.get_parameter('hough_min_line_length').value
        self.hough_max_line_gap = self.get_parameter('hough_max_line_gap').value
        self.morph_kernel_size = self.get_parameter('morph_kernel_size').value
        self.morph_iterations = self.get_parameter('morph_iterations').value
        self.min_circle_radius = self.get_parameter('min_circle_radius').value
        self.max_circle_radius = self.get_parameter('max_circle_radius').value
        
        # Create publishers and subscribers
        self._create_publishers()
        self._create_subscribers()
        
        # Utils
        self.bridge = CvBridge()
        self.get_logger().info('Lane/Obstacle Detector Node initialized')
    
    def _create_subscribers(self):
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
    
    def _create_publishers(self):
        self.mask_publisher = self.create_publisher(
            Image,
            '/perception/lane_obstacle_mask',
            10
        )
        self.debug_preprocessed_publisher = self.create_publisher(
            Image,
            '/perception/debug/preprocessed',
            10
        )
        self.debug_lane_publisher = self.create_publisher(
            Image,
            '/perception/debug/lane_detection',
            10
        )
        self.debug_obstacle_publisher = self.create_publisher(
            Image,
            '/perception/debug/obstacle_detection',
            10
        )
    
    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Save original image dimensions
            original_shape = cv_image.shape
            
            # Preprocess image
            preprocessed_img = self.preprocess_image(cv_image)
            
            # Detect white regions
            white_mask = self.detect_white_regions(preprocessed_img)
            
            # Apply morphological operations
            cleaned_mask = self.apply_morphological_operations(white_mask)
            
            # Detect lanes and obstacles
            lane_mask = self.detect_lanes(cleaned_mask)
            obstacle_mask = self.detect_obstacles(cleaned_mask)
            
            # Create final binary mask
            final_mask = self.create_binary_mask(lane_mask, obstacle_mask, original_shape)
            
            # Publish results
            self.publish_results(final_mask, preprocessed_img, lane_mask, obstacle_mask, msg.header)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
    
    def preprocess_image(self, image):
        # Get image dimensions
        height, width = image.shape[:2]
        
        # Crop sky (top 30%)
        crop_height = int(height * (self.roi_top_crop_percent / 100.0))
        cropped_img = image[crop_height:, :]
        
        # Downsample
        new_height = int(cropped_img.shape[0] * self.downsample_factor)
        new_width = int(cropped_img.shape[1] * self.downsample_factor)
        downsized = cv2.resize(cropped_img, (new_width, new_height))
        
        # Convert to HSV
        hsv_img = cv2.cvtColor(downsized, cv2.COLOR_BGR2HSV)
        
        return hsv_img
    
    def detect_white_regions(self, hsv_image):
        # Apply white thresholding
        white_mask = cv2.inRange(hsv_image, self.white_lower, self.white_upper)
        return white_mask
    
    def apply_morphological_operations(self, binary_image):
        # Create kernel for morphological operations
        kernel = np.ones((self.morph_kernel_size, self.morph_kernel_size), np.uint8)
        
        # Apply dilation to fill small gaps
        dilated = cv2.dilate(binary_image, kernel, iterations=self.morph_iterations)
        
        # Apply erosion to remove noise
        eroded = cv2.erode(dilated, kernel, iterations=self.morph_iterations)
        
        return eroded
    
    def detect_lanes(self, preprocessed_image):
        # Create a copy of the image for lane detection
        lane_img = preprocessed_image.copy()
        
        # Apply Canny edge detection
        edges = cv2.Canny(lane_img, self.canny_low, self.canny_high)
        
        # Apply Hough Line Transform
        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi/180,
            threshold=self.hough_threshold,
            minLineLength=self.hough_min_line_length,
            maxLineGap=self.hough_max_line_gap
        )
        
        # Create an empty mask for detected lines
        lane_mask = np.zeros_like(preprocessed_image)
        
        # Draw detected lines on the mask
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(lane_mask, (x1, y1), (x2, y2), 255, 5)
        
        return lane_mask
    
    def detect_obstacles(self, preprocessed_image):
        # Create a copy of the image for obstacle detection
        obstacle_img = preprocessed_image.copy()
        
        # Apply GaussianBlur to reduce noise
        blurred = cv2.GaussianBlur(obstacle_img, (5, 5), 0)
        
        # Apply Hough Circle Transform
        circles = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            dp=1,
            minDist=20,
            param1=self.canny_high,
            param2=self.hough_threshold,
            minRadius=self.min_circle_radius,
            maxRadius=self.max_circle_radius
        )
        
        # Create an empty mask for detected obstacles
        obstacle_mask = np.zeros_like(preprocessed_image)
        
        # Draw detected circles on the mask
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                center = (i[0], i[1])
                radius = i[2]
                cv2.circle(obstacle_mask, center, radius, 255, -1)  # -1 fills the circle
        
        return obstacle_mask
    
    def create_binary_mask(self, lane_mask, obstacle_mask, original_shape):
        # Combine lane and obstacle masks
        combined_mask = cv2.bitwise_or(lane_mask, obstacle_mask)
        
        # Resize back to original dimensions (considering the cropped sky)
        original_height, original_width = original_shape[:2]
        crop_height = int(original_height * (self.roi_top_crop_percent / 100.0))
        resized_height = original_height - crop_height
        
        resized_mask = cv2.resize(combined_mask, (original_width, resized_height))
        
        # Create a full-sized mask with sky set to false (0)
        full_mask = np.zeros((original_height, original_width), dtype=np.uint8)
        full_mask[crop_height:, :] = resized_mask
        
        return full_mask
    
    def publish_results(self, final_mask, preprocessed_img, lane_mask, obstacle_mask, header):
        # Publish the binary mask
        mask_msg = self.bridge.cv2_to_imgmsg(final_mask, encoding='mono8')
        mask_msg.header = header
        self.mask_publisher.publish(mask_msg)
        
        # Publish debug images
        if self.debug_preprocessed_publisher.get_subscription_count() > 0:
            # Convert HSV back to BGR for visualization
            debug_preproc = cv2.cvtColor(preprocessed_img, cv2.COLOR_HSV2BGR)
            debug_msg = self.bridge.cv2_to_imgmsg(debug_preproc, encoding='bgr8')
            debug_msg.header = header
            self.debug_preprocessed_publisher.publish(debug_msg)
        
        if self.debug_lane_publisher.get_subscription_count() > 0:
            lane_msg = self.bridge.cv2_to_imgmsg(lane_mask, encoding='mono8')
            lane_msg.header = header
            self.debug_lane_publisher.publish(lane_msg)
        
        if self.debug_obstacle_publisher.get_subscription_count() > 0:
            obstacle_msg = self.bridge.cv2_to_imgmsg(obstacle_mask, encoding='mono8')
            obstacle_msg.header = header
            self.debug_obstacle_publisher.publish(obstacle_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()