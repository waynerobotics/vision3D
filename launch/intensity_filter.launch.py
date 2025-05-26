from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Declare launch arguments
    intensity_threshold_arg = DeclareLaunchArgument(
        'intensity_threshold',
        default_value='140.0',
        description='Threshold value for intensity filtering'
    )
    
    input_topic_arg = DeclareLaunchArgument(
        'input_cloud_topic',
        default_value='/unilidar/cloud',
        description='Input point cloud topic'
    )
    
    output_topic_arg = DeclareLaunchArgument(
        'output_cloud_topic',
        default_value='/filtered_pointcloud',
        description='Output filtered point cloud topic'
    )
    
    filter_mode_arg = DeclareLaunchArgument(
        'filter_mode',
        default_value='keep_high',
        description='Filter mode: "above" to keep points below threshold, "below" to keep points above threshold'
    )
    
    # Intensity filter node
    intensity_filter_node = Node(
        package='vision3D',
        executable='intensity_filter',
        name='intensity_filter',
        parameters=[{
            'intensity_threshold': LaunchConfiguration('intensity_threshold'),
            'input_cloud_topic': LaunchConfiguration('input_cloud_topic'),
            'output_cloud_topic': LaunchConfiguration('output_cloud_topic'),
            'filter_mode': LaunchConfiguration('filter_mode')
        }],
        output='screen'
    )
    
    return LaunchDescription([
        intensity_threshold_arg,
        input_topic_arg,
        output_topic_arg,
        filter_mode_arg,
        intensity_filter_node
    ])