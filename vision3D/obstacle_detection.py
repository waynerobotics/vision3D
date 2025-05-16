import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import torch
import numpy as np
from transformers import AutoImageProcessor, SegformerForSemanticSegmentation
import math

# --- Configuration & Parameters ---
FINETUNED_MODEL_PATH = "/home/siva/ros2_ws/src/Shanti_2025/vision3D/vision3D/finetuned_model/"
INPUT_IMAGE_TOPIC = 'camera/image_raw' # Topic from img_pub.py (default)
OUTPUT_PROCESSED_TOPIC = 'lane_detection/image_processed'
OUTPUT_MASK_TOPIC = 'lane_detection/segmentation_mask'

# --- Static Parameters (from original trackbar initials) ---
canny_low = 50
canny_high = 150
hough_thresh = 20
hough_min_len = 20
hough_max_gap = 5
hough_rho = 1              # Distance resolution in pixels
hough_theta = np.pi / 180  # Angular resolution in radians
min_line_angle_deg = 20    # Minimum angle (degrees) from horizontal
max_line_angle_deg = 80    # Maximum angle (degrees) from horizontal

# --- Helper Functions ---
def filter_lines_by_angle(lines, min_angle_deg, max_angle_deg):
    filtered_lines = []
    if lines is None:
        return []
    for line in lines:
        x1, y1, x2, y2 = line[0]
        if x2 - x1 == 0: # Avoid division by zero for vertical lines
            angle_deg = 90
        else:
            angle_rad = math.atan2(abs(y2 - y1), abs(x2 - x1))
            angle_deg = math.degrees(angle_rad)

        # Keep lines that are reasonably steep (filter horizontal/near-horizontal)
        if min_angle_deg <= angle_deg <= max_angle_deg:
            filtered_lines.append(line)
    return filtered_lines

def draw_lines(image, lines, color=(0, 0, 255), thickness=3):
    line_image = np.zeros_like(image)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(line_image, (x1, y1), (x2, y2), color, thickness)
    return line_image

class ObstacleDetectorNode(Node):
    def __init__(self):
        super().__init__('obstacle_detector_node')
        self.get_logger().info('Initializing Obstacle Detector Node...')

        # --- Load Model ---
        self.get_logger().info("Loading model...")
        try:
            # Explicitly disable fast tokenizer usage if causing issues, though default should be fine.
            # self.processor = AutoImageProcessor.from_pretrained(FINETUNED_MODEL_PATH, use_fast=False)
            self.processor = AutoImageProcessor.from_pretrained(FINETUNED_MODEL_PATH)
            self.model = SegformerForSemanticSegmentation.from_pretrained(FINETUNED_MODEL_PATH)
            self.get_logger().info("Model loaded.")
            self.model.eval()
            self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
            self.model.to(self.device)
            self.get_logger().info(f"Using device: {self.device}")
        except Exception as e:
            self.get_logger().error(f"Failed to load model from {FINETUNED_MODEL_PATH}: {e}")
            # Optionally raise the error or handle differently
            rclpy.shutdown() # Shutdown ROS if model fails critical loading
            raise e # Reraise exception to prevent node from continuing in broken state


        # --- ROS Communication ---
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image,
            INPUT_IMAGE_TOPIC,
            self.image_callback,
            10) # QoS profile depth 10
        self.processed_pub = self.create_publisher(Image, OUTPUT_PROCESSED_TOPIC, 10)
        self.mask_pub = self.create_publisher(Image, OUTPUT_MASK_TOPIC, 10)
        self.get_logger().info(f"Subscribed to {INPUT_IMAGE_TOPIC}")
        self.get_logger().info(f"Publishing processed images to {OUTPUT_PROCESSED_TOPIC}")
        self.get_logger().info(f"Publishing segmentation masks to {OUTPUT_MASK_TOPIC}")

        # Store fixed parameters (consider making these ROS parameters later)
        self.canny_low = canny_low
        self.canny_high = canny_high
        self.hough_thresh = hough_thresh
        self.hough_min_len = hough_min_len
        self.hough_max_gap = hough_max_gap
        self.hough_rho = hough_rho
        self.hough_theta = hough_theta
        self.min_line_angle_deg = min_line_angle_deg
        self.max_line_angle_deg = max_line_angle_deg

        self.get_logger().info('Obstacle Detector Node initialized successfully.')

    def image_callback(self, msg):
        self.get_logger().debug(f'Received image on {self.image_sub.topic_name}')
        try:
            # Convert ROS Image message to OpenCV image (BGR format)
            cv_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error converting image: {e}')
            return
        except Exception as e:
             self.get_logger().error(f'Unexpected error converting image: {e}')
             return

        # Process the frame using the model and Hough transform
        try:
             processed_frame, segmented_mask = self.process_frame(cv_frame)

             # Convert segmentation mask (grayscale) back to ROS Image message and publish
             mask_msg = self.bridge.cv2_to_imgmsg(segmented_mask, encoding='mono8')
             mask_msg.header = msg.header # Preserve timestamp and frame_id
             self.mask_pub.publish(mask_msg)
             self.get_logger().debug(f'Published segmentation mask to {self.mask_pub.topic_name}')

             # Convert processed frame to ROS Image message and publish
             processed_msg = self.bridge.cv2_to_imgmsg(processed_frame, encoding='bgr8')
             processed_msg.header = msg.header  # Preserve timestamp and frame_id
             self.processed_pub.publish(processed_msg)
             self.get_logger().debug(f'Published processed image to {self.processed_pub.topic_name}')


        except Exception as e:
            # Log detailed error including traceback
            self.get_logger().error(f"Error during frame processing or publishing: {e}", exc_info=True)


    def process_frame(self, frame):
        # Get frame dimensions directly from the input frame
        if frame is None:
             self.get_logger().warning("process_frame received a None frame.")
             # Return a dummy frame or handle appropriately
             return np.zeros((100, 100, 3), dtype=np.uint8), np.zeros((100, 100), dtype=np.uint8)

        frame_height, frame_width = frame.shape[:2]
        self.get_logger().debug(f"Processing frame with shape: {(frame_height, frame_width)}")


        # 1. Model Inference
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        inputs = self.processor(images=rgb_frame, return_tensors="pt").to(self.device)
        with torch.no_grad():
            outputs = self.model(**inputs)
        logits = outputs.logits
        # Interpolate logits to original frame size
        upsampled_logits = torch.nn.functional.interpolate(
            logits, size=(frame_height, frame_width), mode='bilinear', align_corners=False
        )
        # Get segmentation mask
        mask = upsampled_logits.argmax(dim=1).squeeze().cpu().numpy().astype(np.uint8)

        # 2. Prepare segmentation mask for Hough transform and publishing
        # Use label 1 (assuming it's the lane) for Hough, publish the scaled mask
        # Adjust this based on your model's label IDs if needed
        lane_mask = np.where(mask == 1, 255, 0).astype(np.uint8) # Binary mask for label 1 (lanes?)
        segmented_full_viz = (mask * (255 // (mask.max() if mask.max() > 0 else 1))).astype(np.uint8) # Visualize all classes


        # --- Optional: Combine with Canny ---
        # Canny edge detection can sometimes help refine segmentation boundaries for Hough.
        # However, it might add noise. Using only the segmentation mask is often cleaner.
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, self.canny_low, self.canny_high)
        process_mask = cv2.bitwise_and(lane_mask, edges) # Apply Hough only on edges within the lane mask
        process_mask = lane_mask # Use only the lane segmentation mask for Hough lines
        # --- End Optional Canny ---

        # 3. Hough Line Transform using fixed parameters on the lane mask
        lines = cv2.HoughLinesP(
            process_mask,
            self.hough_rho,
            self.hough_theta,
            self.hough_thresh,
            np.array([]),
            minLineLength=self.hough_min_len,
            maxLineGap=self.hough_max_gap
        )

        # 4. Filter lines based on angle
        filtered_lines = filter_lines_by_angle(lines, self.min_line_angle_deg, self.max_line_angle_deg)

        # 5. Draw filtered lines on a blank image first
        line_image = draw_lines(frame, filtered_lines, color=(0, 255, 0), thickness=5) # Pass frame for size matching

        # 6. Combine line image with original frame
        # Ensure line_image is BGR if frame is BGR
        if len(frame.shape) == 3 and len(line_image.shape) == 2:
             # This might happen if draw_lines created a grayscale mask
             line_image = cv2.cvtColor(line_image, cv2.COLOR_GRAY2BGR)
        elif len(frame.shape) == 2 and len(line_image.shape) == 3:
             # This case means input frame was grayscale, but lines are color? Unlikely with bgr8 input.
             self.get_logger().warning("Input frame is grayscale but line image is color. Converting frame to BGR.")
             frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        elif len(frame.shape) == 3 and len(line_image.shape) == 3 and frame.shape[2] != line_image.shape[2]:
             self.get_logger().error(f"Channel mismatch: Frame {frame.shape}, Line Image {line_image.shape}. Cannot combine.")
             output_frame = frame # Return original frame on error
             return output_frame, segmented_full_viz # Return original frame and mask


        # Add the lines to the original frame
        output_frame = cv2.addWeighted(frame, 0.8, line_image, 1.0, 0.0)

        # Return the combined frame and the visualization mask
        return output_frame, segmented_full_viz

# --- Main Execution ---
def main(args=None):
    print("Initializing ROS 2 node for obstacle detection...")
    rclpy.init(args=args)
    obstacle_detector_node = None # Initialize to None
    try:
        obstacle_detector_node = ObstacleDetectorNode()
        if rclpy.ok(): # Check if node initialization was successful (didn't raise exception)
             print("Obstacle Detector Node spinning...")
             rclpy.spin(obstacle_detector_node)
    except KeyboardInterrupt:
        print('Keyboard interrupt received, shutting down.')
    except Exception as e:
        # Log any exceptions raised during node initialization or spin
        if obstacle_detector_node:
             obstacle_detector_node.get_logger().fatal(f"Unhandled exception in node: {e}", exc_info=True)
        else:
             print(f"Failed to initialize node or encountered error before spinning: {e}")
    finally:
        # Ensure cleanup happens regardless of how spin ended
        if obstacle_detector_node is not None and rclpy.ok():
             print("Destroying node...")
             obstacle_detector_node.destroy_node()
        if rclpy.ok():
             print("Shutting down rclpy...")
             rclpy.shutdown()
        print("Cleanup finished.")


if __name__ == '__main__':
    main()
