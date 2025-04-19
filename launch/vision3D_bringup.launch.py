from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():


    # Image publisher node
    image_publisher_node = Node(
        package='vision3D',
        executable='img_pub',
        name='image_publisher',
        parameters=[{
            'video_source': 0,
            'image_topic': 'camera/image_raw',
        }],
        output='screen',
    )

    return LaunchDescription([
        # video_source_arg,
        # image_topic_arg,
        image_publisher_node,
    ])