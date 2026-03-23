import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import SetParameter
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_bringup = get_package_share_directory('demeter_bringup')

    # --- Arguments ---
    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')

    declare_world = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_bringup, 'worlds', 'marsyard2020_walls.sdf'),
        description='Path to the Gazebo world file (.sdf)'
    )

    declare_headless = DeclareLaunchArgument(
        'headless',
        default_value='false',
        choices=['true', 'false'],
        description='Start Gazebo in headless mode'
    )

    # --- Gazebo Sim (GUI) ---
    gz_gui = PythonLaunchDescriptionSource(
        os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
    )

    gz_sim_with_gui = {
        'gz_version': '6',
        'gz_args': ['-r ', '-v 4 ', world]
    }

    gz_sim_headless = {
        'gz_version': '6',
        'gz_args': ['-r ', '-v 4 ', '-s ', world]
    }

    gazebo_gui = IncludeLaunchDescription(
        gz_gui,
        launch_arguments=gz_sim_with_gui.items(),
        condition=UnlessCondition(headless)
    )

    gazebo_headless = IncludeLaunchDescription(
        gz_gui,
        launch_arguments=gz_sim_headless.items(),
        condition=IfCondition(headless)
    )

    return LaunchDescription([
        declare_world,
        declare_headless,
        gazebo_gui,
        gazebo_headless,
    ])