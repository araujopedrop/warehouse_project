import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution

def generate_launch_description():

    controller_yaml   = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'controller.yaml')
    bt_navigator_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'bt_navigator.yaml')
    planner_yaml      = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'planner_server.yaml')
    recovery_yaml     = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'recovery.yaml')
    rviz_config_dir   = os.path.join(get_package_share_directory("path_planner_server"), "config", 'path_planner_rviz_config.rviz')

    
    # ************************* Arguments *************************
    
    use_sim_time_value_default = "True"

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value=use_sim_time_value_default
    )

    use_sim_time_value = LaunchConfiguration('use_sim_time')

    # ************************* Arguments *************************

    

    # See if I will use sim or real robot
    # Not only use_sim_time parameters I have to change, also odom_frame_id
    def compare_arg(context):
        use_sim_time_value_ = use_sim_time_value.perform(context)

        if use_sim_time_value_.lower() == "true":

            controller_node = Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                parameters=[controller_yaml, {'use_sim_time': use_sim_time_value}, {'cmd_vel_topic': "/diffbot_base_controller/cmd_vel_unstamped"}, {'global_frame': "odom"}],
                remappings=[('/cmd_vel', '/diffbot_base_controller/cmd_vel_unstamped')])

            planner_node = Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[planner_yaml, {'use_sim_time': use_sim_time_value}],
                remappings=[
                ('/cmd_vel', '/diffbot_base_controller/cmd_vel_unstamped')
                ])

            behavior_node = Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                parameters=[recovery_yaml, {'use_sim_time': use_sim_time_value}, {'global_frame': "odom"}],
                output='screen',
                remappings=[
                ('/cmd_vel', '/diffbot_base_controller/cmd_vel_unstamped')
                ])

            bt_navigator_node = Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[bt_navigator_yaml, {'use_sim_time': use_sim_time_value}],
                remappings=[
                    ('/cmd_vel', '/diffbot_base_controller/cmd_vel_unstamped')
                ])

            lifecycle_manager_node = Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_pathplanner',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time_value},
                            {'autostart': True},
                            {'node_names': ['planner_server',
                                            'controller_server',
                                            'bt_navigator',
                                            'behavior_server']}])
                


        elif use_sim_time_value_.lower() == "false":

            controller_node = Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                parameters=[controller_yaml, {'use_sim_time': use_sim_time_value}, {'cmd_vel_topic': "/cmd_vel"}, {'global_frame': "robot_odom"}],
                remappings=[('/cmd_vel', '/cmd_vel')])

            planner_node = Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[planner_yaml, {'use_sim_time': use_sim_time_value}],
                remappings=[
                ('/cmd_vel', '/cmd_vel')
                ])

            behavior_node = Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                parameters=[recovery_yaml, {'use_sim_time': use_sim_time_value}, {'global_frame': "robot_odom"}],
                output='screen',
                remappings=[
                ('/cmd_vel', '/cmd_vel')
                ])

            bt_navigator_node = Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[bt_navigator_yaml, {'use_sim_time': use_sim_time_value}],
                remappings=[
                    ('/cmd_vel', '/cmd_vel')
                ])

            lifecycle_manager_node = Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_pathplanner',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time_value},
                            {'autostart': True},
                            {'node_names': ['planner_server',
                                            'controller_server',
                                            'bt_navigator',
                                            'behavior_server']}])



        return [controller_node, planner_node, behavior_node, bt_navigator_node, lifecycle_manager_node]


    compare_action = OpaqueFunction(function=compare_arg)
    
    return LaunchDescription([     

        use_sim_time_arg,

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir]),

        compare_action,
    ])
