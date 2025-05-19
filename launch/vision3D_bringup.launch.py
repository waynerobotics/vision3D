from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('vision3D')
    model_path = os.path.join(pkg_share, 'finetuned_model')
    
    # Declare launch arguments
    video_source_arg = DeclareLaunchArgument(
        'video_source',
        default_value='0',
        description='Video source for the camera (device number or file path)'
    )
    
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='camera/image_raw',
        description='Topic name for publishing camera images'
    )
    
    processed_topic_arg = DeclareLaunchArgument(
        'processed_topic',
        default_value='lane_detection/image_processed',
        description='Topic name for publishing processed images'
    )
    
    mask_topic_arg = DeclareLaunchArgument(
        'mask_topic',
        default_value='lane_detection/segmentation_mask',
        description='Topic name for publishing segmentation mask'
    )

    # Image publisher node
    image_publisher_node = Node(
        package='vision3D',
        executable='img_pub',
        name='image_publisher',
        parameters=[{
            'video_source': LaunchConfiguration('video_source'),
            'image_topic': LaunchConfiguration('image_topic'),
            'model_path': model_path,
        }],
        output='screen',
    )
    
    # Obstacle detection node
    obstacle_detector_node = Node(
        package='vision3D',
        executable='obstacle_detection',
        name='obstacle_detector',
        parameters=[{
            'model_path': model_path,
            'input_topic': LaunchConfiguration('image_topic'),
            'processed_topic': LaunchConfiguration('processed_topic'),
            'mask_topic': LaunchConfiguration('mask_topic'),
        }],
        output='screen',
    )

    return LaunchDescription([
        video_source_arg,
        image_topic_arg,
        processed_topic_arg,
        mask_topic_arg,
        image_publisher_node,
        obstacle_detector_node,
    ])