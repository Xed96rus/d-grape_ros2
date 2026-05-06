from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition, UnlessCondition

from ament_index_python.packages import get_package_share_directory

import os



def generate_launch_description():
    ## Arguments
    pkg_description = get_package_share_directory('d-grape_description')
    sim_arg = DeclareLaunchArgument(name='sim', default_value='true', choices=['true', 'false'],
                                    description='Set to true to switch from hardware to simulation in the loop')
    robot_name_arg = DeclareLaunchArgument(name='robot_name', default_value='d-grape',
                                    description='Unique robot name')
    namespace_arg = DeclareLaunchArgument(name='use_namespace', default_value="false", choices=['true', 'false'],
                                    description='Use robot name to namespace robot')
    
    robot_name = LaunchConfiguration('robot_name')
    xacro_file = os.path.join(pkg_description, 'models', 'urdf', 'd-grape.urdf.xacro')

    ## Launch group
    description_group_with_ns = GroupAction(
        condition=IfCondition(LaunchConfiguration('use_namespace')),
        actions = [
        # URDF with namespace
            PushRosNamespace(robot_name),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                parameters=[{
                    'use_sim_time' : LaunchConfiguration('sim'),
                    'frame_prefix': [robot_name, '/'],
                    'robot_description': ParameterValue(
                        Command([
                            'xacro ', xacro_file,
                            ' sim:=', LaunchConfiguration('sim'),
                            ' robot_name:=', robot_name,
                            ' namespace:=', robot_name,
                            ' frame_prefix:=', '/'
                        ]), value_type=str
                    )
                }],
                remappings=[('/tf', 'tf'), 
                            ('/tf_static', 'tf_static')],
                output='screen'
            ),

    ])
    description_group_no_ns = GroupAction(
        condition=UnlessCondition(LaunchConfiguration('use_namespace')),
        actions= [

        # URDF without namespace
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                parameters=[{
                    'use_sim_time' : LaunchConfiguration('sim'),
                    'robot_description': ParameterValue(
                        Command([
                            'xacro ', xacro_file,
                            ' sim:=', LaunchConfiguration('sim'),
                            ' robot_name:=', robot_name,
                            ' namespace:=', '/',
                            ' frame_prefix:=', '/'
                        ]), value_type=str
                    )
                }],
                remappings=[('/tf', 'tf'), 
                            ('/tf_static', 'tf_static')],
                output='screen'
            ),

    ])


    ## Launch description
    return LaunchDescription([

        # Arguments
        sim_arg,
        robot_name_arg,
        namespace_arg,

        # Launch
        description_group_with_ns,
        description_group_no_ns
    ])
    