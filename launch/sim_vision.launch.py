from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Image publisher
    image_publisher_node = Node(
        package='vision3D',
        executable='img_pub',
        name='image_publisher'
    )

    # Test Image publisher
    test_image_publisher_node = Node(
        package='vision3D',
        executable='test_img_pub',
        name='test_image_publisher',
        remappings=[('/image_raw', '/image_raw')]
    )

    # Obstacle detection
    obstacle_detector_node = Node(
        package='vision3D',
        executable='obstacle_detection',
        name='obstacle_detector',
        remappings=[('/image_raw', '/image_raw'), ('/obstacle_mask', '/obstacle_mask')]
    )

    # Lane segmentation
    lane_segmentation_node = Node(
        package='vision3D',
        executable='lane_segmentation',
        name='lane_segmentation',
        remappings=[('/obstacle_mask', '/obstacle_mask'), ('/lane_pointcloud', '/lane_pointcloud')]
    )

    return LaunchDescription([
        # image_publisher_node,
        test_image_publisher_node,
        obstacle_detector_node,
        lane_segmentation_node
    ])