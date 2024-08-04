import os

from launch_ros.actions          import Node
from launch                      import LaunchDescription
from launch.actions              import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions        import LaunchConfiguration, TextSubstitution, PathJoinSubstitution

from ament_index_python.packages import get_package_share_directory

use_sim_time_parameter = True

def generate_launch_description():
    
    nav2_yaml = os.path.join(get_package_share_directory('localization_server'), 'config', 'amcl_config.yaml')
    
    map_file_default = "warehouse_map_sim.yaml"

    map_file_arg = DeclareLaunchArgument(
        "map_file", default_value=map_file_default
    )
    
    # Define the base directory
    base_dir = os.path.join(get_package_share_directory('map_server'), "config")

    # Concatenate the base directory with the file name
    map_file_path = PathJoinSubstitution([
        TextSubstitution(text=base_dir),
        LaunchConfiguration('map_file')
    ])

    rviz_config_dir = os.path.join(get_package_share_directory("localization_server"), 'config', 'localizer_rviz_config.rviz')

    return LaunchDescription([

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_base_link',
            output='screen',
            emulate_tty=True,
            arguments=['0', '0', '0', '0', '0', '0', 'robot_base_link', 'base_link']),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml]),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True}, 
                        {'yaml_filename':map_file_path} 
                       ]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server','amcl']}]),

        map_file_arg,
        ])