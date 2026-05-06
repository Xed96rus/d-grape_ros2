from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition

from ament_index_python.packages import get_package_share_directory

import os



def generate_launch_description():
    ## Arguments
    robot_name_arg = DeclareLaunchArgument(name='robot_name', default_value='d-grape',
                                    description='Unique robot name')
    namespace_arg = DeclareLaunchArgument(name='use_namespace', default_value="false", choices=['true', 'false'],
                                    description='Use robot name to namespace robot')
    rviz_arg = DeclareLaunchArgument(name='rviz', default_value="false", choices=['true', 'false'],
                                    description='Bringup rviz')
    
    pkg_description = get_package_share_directory('d-grape_description')
    ## Launch group
    bringup_group_with_ns = GroupAction(
        condition=IfCondition(LaunchConfiguration('use_namespace')),
        actions = [
        PushRosNamespace(LaunchConfiguration('robot_name')),
        # URDF with namespace
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(pkg_description, 'launch', 'description.launch.py')]),
            launch_arguments={
                'robot_name': LaunchConfiguration('robot_name'),
                'use_namespace': LaunchConfiguration('use_namespace'),
                'sim': 'false',
            }.items()
        ),

        # Launch control for sim
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('d-grape_sim_control'), 'launch', 'control.launch.py'])),
            condition = IfCondition(LaunchConfiguration('sim')),
            launch_arguments={
                'robot_name': LaunchConfiguration('robot_name'),
                'use_namespace': LaunchConfiguration('use_namespace'),
                'sim': LaunchConfiguration('sim'),
            }.items()
        ),

        # 3 stereo cam hardware
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('d-grape_camera_hardware'), 'launch', 'stereo_cameras.launch.py'])),
            condition = UnlessCondition(LaunchConfiguration('sim')),
        ),

        # Micro-ROS agent for hardware
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=['serial', '--dev', '/dev/ttyACM1', '-b', '115200'],
            condition=UnlessCondition(LaunchConfiguration('sim')),
            output='screen'
        ),
    ])
    bringup_group_no_ns = GroupAction(
        condition=UnlessCondition(LaunchConfiguration('use_namespace')),
        actions = [

        # URDF without namespace
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(pkg_description, 'launch', 'description.launch.py')]),
                launch_arguments={
                    'robot_name': LaunchConfiguration('robot_name'),
                    'use_namespace': LaunchConfiguration('use_namespace'),
                    'sim': 'false',
                }.items()
            ),

        # Launch control for sim
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('d-grape_sim_control'), 'launch', 'control.launch.py'])),
            condition = IfCondition(LaunchConfiguration('sim')),
            launch_arguments={
                'robot_name': LaunchConfiguration('robot_name'),
                'use_namespace': LaunchConfiguration('use_namespace'),
                'sim': LaunchConfiguration('sim'),
            }.items()
        ),

        # 3 stereo cam hardware
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('d-grape_camera_hardware'), 'launch', 'stereo_cameras.launch.py'])),
            condition = UnlessCondition(LaunchConfiguration('sim')),
        ),

        # Micro-ROS agent for hardware
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=['serial', '--dev', '/dev/ttyACM1', '-b', '115200'],
            condition=UnlessCondition(LaunchConfiguration('sim')),
            output='screen'
        ),
    ])


    ## Launch description
    return LaunchDescription([

        # Arguments
        robot_name_arg,
        namespace_arg,
        rviz_arg,

        # Launch
        bringup_group_with_ns,
        bringup_group_no_ns
    ])
    