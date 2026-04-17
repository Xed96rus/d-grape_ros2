from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

from ament_index_python.packages import get_package_share_directory

import os

robot = 'demeter'

def generate_launch_description():
    pkg_bringup = get_package_share_directory('demeter_bringup')
    ## Arguments
    sim_arg = DeclareLaunchArgument(name='sim', default_value='true', choices=['true', 'false'],
                                    description='Set to true to switch from hardware to simulation in the loop')
    robot_name_arg = DeclareLaunchArgument(name='robot_name', default_value=robot,
                                    description='Unique robot name')
    namespace_arg = DeclareLaunchArgument(name='use_namespace', default_value="false", choices=['true', 'false'],
                                    description='Use robot name to namespace robot')
    world_arg = DeclareLaunchArgument(name='world', default_value=os.path.join(pkg_bringup, 'worlds', 'marsyard2020_walls.sdf'),
                                    description='Gazebo world path')
    declare_headless = DeclareLaunchArgument(
        'headless',
        default_value='false',
        choices=['true', 'false'],
        description='Start Gazebo in headless mode'
    )
    ## Simulation specific arguments
    x_arg = DeclareLaunchArgument(name='x', default_value='0', description='[Simulation only] x-position')
    y_arg = DeclareLaunchArgument(name='y', default_value='0', description='[Simulation only] y-position')
    

    ## Launch group
    launch_group = GroupAction([

        # Launch Gazebo world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(robot+'_bringup'), 'launch', 'gazebo_world.launch.py')]),
            condition=IfCondition(LaunchConfiguration('sim')),
            launch_arguments={
                'headless': LaunchConfiguration('headless'),
                'world': LaunchConfiguration('world'),
            }.items()
        ),

        # Spawn robot in gazebo world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(robot+'_bringup'), 'launch', 'spawn_robot.launch.py')]),
            condition=IfCondition(LaunchConfiguration('sim')),
            launch_arguments={
                'robot_name': LaunchConfiguration('robot_name'),
                'use_namespace': LaunchConfiguration('use_namespace'),
                'x': LaunchConfiguration('x'),
                'y': LaunchConfiguration('y'),
                
            }.items()
        ),

        # Launch control
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(robot+'_control'), 'launch', 'control.launch.py')]),
            launch_arguments={
                'robot_name': LaunchConfiguration('robot_name'),
                'use_namespace': LaunchConfiguration('use_namespace'),
                'sim': LaunchConfiguration('sim'),
            }.items()
        ),
    ])



    ## Launch description
    return LaunchDescription([

        # Arguments
        sim_arg,
        robot_name_arg,
        namespace_arg,
        world_arg,
        declare_headless,
        x_arg,
        y_arg,

        # Launch
        launch_group
    ])
    