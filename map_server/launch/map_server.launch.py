import os
from launch_ros.actions          import Node
from launch                      import LaunchDescription
from launch.actions              import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.substitutions        import LaunchConfiguration, TextSubstitution, PathJoinSubstitution

def generate_launch_description():
    
    # RVIZ Configuration
    rviz_config_dir = os.path.join(get_package_share_directory("map_server"), "config", 'mapper_rviz_config.rviz')

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

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True}, 
                        {'topic_name':"map"},
                        {'frame_id':"map"},
                        {'yaml_filename':map_file_path} 
                       ]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server']}]),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir]),            

        map_file_arg,
        ])