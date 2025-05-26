#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
import numpy as np

class IntensityFilter(Node):
    def __init__(self):
        super().__init__('intensity_filter')
        
        # Declare parameters
        self.declare_parameter('intensity_threshold', 100.0)
        self.declare_parameter('input_cloud_topic', '/unilidar/cloud')
        self.declare_parameter('output_cloud_topic', '/high_intensity_pointcloud')
        self.declare_parameter('filter_mode', 'keep_high') # 'keep_high' or 'keep_low'
        
        # Get parameters
        self.intensity_threshold = self.get_parameter('intensity_threshold').value
        self.input_topic = self.get_parameter('input_cloud_topic').value
        self.output_topic = self.get_parameter('output_cloud_topic').value
        self.filter_mode = self.get_parameter('filter_mode').value
        
        # Create subscriber
        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.pointcloud_callback,
            10)
        
        # Create publisher
        self.publisher = self.create_publisher(
            PointCloud2,
            self.output_topic,
            10)
        
        self.get_logger().info(f'Intensity filter initialized with threshold: {self.intensity_threshold}')
        self.get_logger().info(f'Filter mode: {self.filter_mode} (keeping {"high" if self.filter_mode == "keep_high" else "low"} intensity points)')
        self.get_logger().info(f'Subscribing to: {self.input_topic}')
        self.get_logger().info(f'Publishing to: {self.output_topic}')
    
    def pointcloud_callback(self, msg):
        try:
            # Extract point cloud data
            pc_data = list(pc2.read_points(msg, 
                                        field_names=("x", "y", "z", "intensity", "ring", "time"), 
                                        skip_nans=True))
            
            if not pc_data:
                self.get_logger().warn('Received empty point cloud')
                return
            
            # Convert to structured numpy array
            points_array = np.array(pc_data, dtype=[
                ('x', np.float32),
                ('y', np.float32),
                ('z', np.float32),
                ('intensity', np.float32),
                ('ring', np.uint16),
                ('time', np.float32)
            ])
            self.get_logger().info(f"points_array intensity: {points_array['intensity']}")
            # Apply intensity filter
            if self.filter_mode == 'keep_high':
                # Keep points with intensity above threshold (high intensity)
                filtered_indices = points_array['intensity'] >= self.intensity_threshold
            elif self.filter_mode == 'keep_low':
                # Keep points with intensity below threshold (low intensity)
                filtered_indices = points_array['intensity'] < self.intensity_threshold
            else:
                self.get_logger().error(f"Invalid filter mode: {self.filter_mode}. Use 'keep_high' or 'keep_low'.")
                return
            self.get_logger().info(f"filtered_indices: {len(filtered_indices)}")
            self.get_logger().info(f"filter_mode: {self.filter_mode}")
            filtered_points = points_array[filtered_indices]
            
            # Create filtered point cloud message
            filtered_points_list = [(p['x'], p['y'], p['z'], p['intensity'], p['ring'], p['time']) 
                              for p in filtered_points]
            
            # Create fields for the filtered point cloud
            fields = [
                pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='intensity', offset=12, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='ring', offset=16, datatype=pc2.PointField.UINT16, count=1),
                pc2.PointField(name='time', offset=20, datatype=pc2.PointField.FLOAT32, count=1),
            ]
            
            # Create filtered point cloud message
            filtered_cloud = pc2.create_cloud(msg.header, fields, filtered_points_list)
            
            # Publish filtered point cloud
            self.publisher.publish(filtered_cloud)
            
            # Log stats
            total_points = len(pc_data)
            filtered_count = len(filtered_points)
            percentage = (filtered_count / total_points) * 100 if total_points > 0 else 0
            
            self.get_logger().info(f'Filtered pointcloud: {filtered_count}/{total_points} points ({percentage:.2f}%) ' + 
                                  f'with intensity {"above" if self.filter_mode == "keep_high" else "below"} {self.intensity_threshold}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {e}')

def main(args=None):
    rclpy.init(args=args)
    intensity_filter = IntensityFilter()
    rclpy.spin(intensity_filter)
    intensity_filter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()