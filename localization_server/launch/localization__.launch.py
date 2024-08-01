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

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="False"
    )

    use_sim_time_value = LaunchConfiguration('use_sim_time')

    # See if I will use sim o real robot
    # Not only use_sim_time parameters I hae to chance, also odom_frame_id
    def compare_arg(context):
        use_sim_time_value_ = use_sim_time_value.perform(context)

        if use_sim_time_value_ == "True":
            # Sim robot
            amcl_node = Node(
                    package='nav2_amcl',
                    executable='amcl',
                    name='amcl',
                    output='screen',
                    parameters=[nav2_yaml, {'use_sim_time': use_sim_time_value}, {'odom_frame_id': "odom"}]
                )
        else:
            # Real robot
            amcl_node = Node(
                    package='nav2_amcl',
                    executable='amcl',
                    name='amcl',
                    output='screen',
                    parameters=[nav2_yaml, {'use_sim_time': use_sim_time_value}, {'odom_frame_id': "robot_odom"}]
                )

        return [amcl_node]

    compare_action = OpaqueFunction(function=compare_arg)


    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time_parameter}, 
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
        use_sim_time_arg,
        compare_action,            
        ])