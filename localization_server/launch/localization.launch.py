import os

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # ************************* Arguments *************************
    
    map_file_default = "warehouse_map_sim.yaml"
    use_sim_time_value_default = "True"

    map_file_arg = DeclareLaunchArgument(
        "map_file", default_value=map_file_default
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value=use_sim_time_value_default
    )

    map_file_LaunchConfiguration = LaunchConfiguration('map_file')
    use_sim_time_value = LaunchConfiguration('use_sim_time')

    # ************************* Arguments *************************

    nav2_yaml = os.path.join(get_package_share_directory('localization_server'), 'config', 'amcl_config.yaml')
    base_dir = os.path.join(get_package_share_directory('map_server'), "config")

    # Concatenate the base directory with the file name
    map_file_path = PathJoinSubstitution([
        TextSubstitution(text=base_dir),
        map_file_LaunchConfiguration
    ])

    # See if I will use sim or real robot
    # Not only use_sim_time parameters I have to change, also odom_frame_id
    def compare_arg(context):
        use_sim_time_value_ = use_sim_time_value.perform(context)

        if use_sim_time_value_.lower() == "true":
            amcl_node = Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                parameters=[nav2_yaml, {'use_sim_time': use_sim_time_value}, {'odom_frame_id': "odom"}]
            )

            static_transform = Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='static_transform_publisher_base_link',
                output='screen',
                emulate_tty=True,
                arguments=['0', '0', '0', '0', '0', '0', 'robot_base_link', 'base_link'])

        elif use_sim_time_value_.lower() == "false":
            amcl_node = Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                parameters=[nav2_yaml, {'use_sim_time': use_sim_time_value}, {'odom_frame_id': "robot_odom"}]
            )

            static_transform = Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='static_transform_publisher_base_link',
                output='screen',
                emulate_tty=True,
                arguments=['0', '0', '0', '0', '0', '0', 'robot_base_link', 'base_link'])

        else:
            amcl_node = Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                parameters=[nav2_yaml, {'use_sim_time': use_sim_time_value}, {'odom_frame_id': "odom"}]
            )

            static_transform = Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='static_transform_publisher_base_link',
                output='screen',
                emulate_tty=True,
                arguments=['0', '0', '0', '0', '0', '0', 'robot_base_link', 'base_link'])

        return [amcl_node, static_transform]

    compare_action = OpaqueFunction(function=compare_arg)

    return LaunchDescription([
        map_file_arg,
        use_sim_time_arg,

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time_value}, 
                        {'yaml_filename': map_file_path}]
        ),

        LogInfo(msg="Launching lifecycle_manager_mapper..."),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time_value},
                        {'autostart': True},
                        {'node_names': ['map_server','amcl']}]
        ),

        compare_action,
    ])
