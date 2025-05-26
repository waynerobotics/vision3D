from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('vision3D')
    
    # Declare launch arguments for the intensity filter
    intensity_threshold_arg = DeclareLaunchArgument(
        'intensity_threshold',
        default_value='100.0',
        description='Threshold value for intensity filtering'
    )
    
    input_cloud_topic_arg = DeclareLaunchArgument(
        'input_cloud_topic',
        default_value='/unilidar/cloud',
        description='Input point cloud topic'
    )
    
    output_cloud_topic_arg = DeclareLaunchArgument(
        'output_cloud_topic',
        default_value='/high_intensity_pointcloud',
        description='Output filtered point cloud topic'
    )
    
    # Include the vision3D_bringup launch file
    vision3d_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 'vision3D_bringup.launch.py')
        ]),
        # Pass any arguments that need to be overridden for vision3D_bringup
        # launch_arguments={
        #     'video_source': LaunchConfiguration('video_source'),
        # }.items()
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
            'filter_mode': 'keep_high'  # Fixed to keep high intensity points
        }],
        output='screen'
    )
    
    return LaunchDescription([
        intensity_threshold_arg,
        input_cloud_topic_arg,
        output_cloud_topic_arg,
        vision3d_launch,  # Launch vision3D components first
        intensity_filter_node  # Then launch intensity filter
    ])