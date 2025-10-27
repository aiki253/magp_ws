import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    share_dir = get_package_share_directory('urg_node2')
    
    rviz_config_file = os.path.join(
        get_package_share_directory('urg_node2'),
        'config',
        'urg_cartographer.rviz')
    
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir',
        default=os.path.join(share_dir, 'config'))
    
    configuration_basename = LaunchConfiguration('configuration_basename', 
        default='magp_2d_localization.lua')
    
    # suzlab20251026ディレクトリ内のmap.pbstreamを使用
    map_filename = LaunchConfiguration('map_filename',
        default='/home/jetson/magp_ws/data/map/suzlab20251027/map.pbstream')
    
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_filename',
            default_value='/home/jetson/magp_ws/data/map/suzlab20251027/map.pbstream',
            description='Full path to the .pbstream map file'
        ),
        
        Node(package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'laser']
        ),
        
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'resolution': 0.05}],
        ),
        
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=[
                '-configuration_directory', cartographer_config_dir, 
                '-configuration_basename', configuration_basename,
                '-load_state_filename', '/home/jetson/magp_ws/data/map/suzlab20251027/map.pbstream'
            ],
        ),
    ])