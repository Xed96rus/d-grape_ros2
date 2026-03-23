# demeter_bringup/launch/rviz.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        choices=['true', 'false'],
        description='Use simulation (Gazebo) clock if true'
    )
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='demeter',
        description='Name of the robot'
    )
    use_namespace_arg = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        choices=['true', 'false'],
        description='Apply robot_name as namespace'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')
    use_namespace = LaunchConfiguration('use_namespace')

    rviz_config_file = PathJoinSubstitution([
        get_package_share_directory('demeter_bringup'),
        'config',
        'demeter.rviz'
    ])

    # --- Без namespace ---
    rviz_no_ns = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=UnlessCondition(use_namespace),
        output='screen'
    )

    # --- С namespace: ГРУППА с PushRosNamespace ---
    rviz_with_ns_group = GroupAction(
        condition=IfCondition(use_namespace),
        actions=[
            PushRosNamespace(robot_name),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config_file, '-f', (robot_name, '/base_link')],
                parameters=[{'use_sim_time': use_sim_time}],
                remappings=[
                    ('/tf', 'tf'),           # ← читать из /demeter/tf
                    ('/tf_static', 'tf_static'),
                    ('/robot_description', 'robot_description')
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        robot_name_arg,
        use_namespace_arg,
        rviz_no_ns,
        rviz_with_ns_group  # ← ВАЖНО: группа, а не отдельный узел
    ])