import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from flask import Flask, Response, request
import threading
import cv2
import numpy as np
import time

class ImageServer(Node):
    def __init__(self):
        super().__init__('image_server')

        # Parameters
        self.declare_parameter('image_topic', 'camera/image_raw')
        self.declare_parameter('max_fps', 10.0)  # Maximum frames per second
        self.declare_parameter('scale_factor', 1.0)  # Image scaling factor (1.0 = original size)
        self.declare_parameter('flask_port', 5001)  # Port for the Flask server
        self.declare_parameter('flask_threads', 4)  # Number of threads for Flask server

        # Get parameter values
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.max_fps = self.get_parameter('max_fps').get_parameter_value().double_value
        self.scale_factor = self.get_parameter('scale_factor').get_parameter_value().double_value
        flask_port = self.get_parameter('flask_port').get_parameter_value().integer_value
        flask_threads = self.get_parameter('flask_threads').get_parameter_value().integer_value

        # Set up frame rate limiter
        self.min_frame_time = 1.0 / self.max_fps if self.max_fps > 0 else 0
        self.last_frame_time = 0

        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.latest_frame = None
        self.frame_lock = threading.Lock()

        # Start Flask server in a separate thread
        self.app = Flask(__name__)
        self.app.add_url_rule('/stream', 'stream', self.stream)
        self.app.add_url_rule('/', 'index', self.index)
        threading.Thread(target=self.run_flask, args=(flask_port, flask_threads), daemon=True).start()

        self.get_logger().info(f"Image server started with max_fps={self.max_fps}, scale_factor={self.scale_factor}")

    def index(self):
        return """
        <html>
            <head>
                <title>ROS2 Image Stream</title>
                <style>
                    body { font-family: Arial, sans-serif; text-align: center; margin-top: 20px; }
                    img { max-width: 90%; }
                    .controls { margin: 20px; }
                </style>
            </head>
            <body>
                <h1>ROS2 Image Stream</h1>
                <img src="/stream" />
                <div class="controls">
                    <p>Stream configuration can be adjusted through ROS2 parameters</p>
                </div>
            </body>
        </html>
        """

    def image_callback(self, msg):
        # Implement frame rate limiting
        current_time = time.time()
        if current_time - self.last_frame_time < self.min_frame_time:
            return
        
        self.last_frame_time = current_time
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Scale image if needed for performance
            if self.scale_factor != 1.0:
                new_width = int(cv_image.shape[1] * self.scale_factor)
                new_height = int(cv_image.shape[0] * self.scale_factor)
                cv_image = cv2.resize(cv_image, (new_width, new_height), 
                                    interpolation=cv2.INTER_AREA if self.scale_factor < 1.0 else cv2.INTER_LINEAR)
            
            with self.frame_lock:
                self.latest_frame = cv_image
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

    def stream(self):
        def generate():
            while True:
                with self.frame_lock:
                    if self.latest_frame is not None:
                        # Use quality parameter to reduce JPEG quality for faster processing
                        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50]
                        _, jpeg = cv2.imencode('.jpg', self.latest_frame, encode_param)
                        frame = jpeg.tobytes()
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
                # Add a small sleep to prevent CPU overuse
                time.sleep(0.01)
                
        return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

    def run_flask(self, port, threads):
        # Use Flask's built-in threaded server for compatibility
        self.get_logger().info(f"Starting Flask server on port {port} with threading enabled")
        self.app.run(host='0.0.0.0', port=port, threaded=True)

def main(args=None):
    rclpy.init(args=args)
    image_server = ImageServer()
    rclpy.spin(image_server)
    image_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()