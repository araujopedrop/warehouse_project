import os

from launch_ros.actions          import Node
from launch                      import LaunchDescription
from launch.actions              import DeclareLaunchArgument, OpaqueFunction
from ament_index_python.packages import get_package_share_directory
from launch.substitutions        import LaunchConfiguration, TextSubstitution, PathJoinSubstitution

def generate_launch_description():

    controller_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'controller.yaml')
    bt_navigator_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'bt_navigator.yaml')
    planner_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'planner_server.yaml')
    recovery_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'recovery.yaml')

    rviz_config_dir = os.path.join(get_package_share_directory("path_planner_server"), "config", 'path_planner_rviz_config.rviz')


    use_sim_time_value = LaunchConfiguration('use_sim_time')

    # See if I will use sim o real robot
    # Not only use_sim_time parameters I hae to chance, also odom_frame_id
    def compare_arg(context):
        use_sim_time_value_ = use_sim_time_value.perform(context)

        if use_sim_time_value_ == "True":
            # Sim robot
            controller_node = Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                parameters=[controller_yaml, {'use_sim_time': use_sim_time_value}, {'cmd_vel_topic': "/diffbot_base_controller/cmd_vel_unstamped"}, {'global_frame': "odom"}])

            planner_node = Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[planner_yaml, {'use_sim_time': use_sim_time_value}])

            behavior_node = Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                parameters=[recovery_yaml, {'use_sim_time': use_sim_time_value}, {'global_frame': "odom"}],
                output='screen')

            bt_navigator = Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[bt_navigator_yaml, {'use_sim_time': use_sim_time_value}])

        else:
            # Real robot
            controller_node = Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                parameters=[controller_yaml, {'use_sim_time': use_sim_time_value}, {'cmd_vel_topic': "/cmd_vel"}, {'global_frame': "robot_odom"}])

            planner_node = Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[planner_yaml, {'use_sim_time': use_sim_time_value}])

            behavior_node = Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                parameters=[recovery_yaml, {'use_sim_time': use_sim_time_value}, {'global_frame': "robot_odom"}],
                output='screen')

            bt_navigator = Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[bt_navigator_yaml, {'use_sim_time': use_sim_time_value}])

        return [controller_node, planner_node, behavior_node, bt_navigator]

    compare_action = OpaqueFunction(function=compare_arg)

    return LaunchDescription([     

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanner',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': ['planner_server',
                                        'controller_server',
                                        'behavior_server',
                                        'bt_navigator']}]),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': False}],
            arguments=['-d', rviz_config_dir]),

        compare_action,

    ])