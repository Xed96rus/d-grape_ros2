from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory

import os



def generate_launch_description():
    
    ## Arguments
    robot_name_arg = DeclareLaunchArgument(name='robot_name', default_value='d-grape',
                                    description='Unique robot name')
    namespace_arg = DeclareLaunchArgument(name='use_namespace', default_value="false", choices=['true', 'false'],
                                    description='Use robot name to namespace robot')
    ## Parameters
    controller_params = os.path.join(get_package_share_directory('d-grape_control'), 'config', 'controller_params.yaml')

    ## Robot model
    # xacro_file = os.path.join(get_package_share_directory('d-grape_description'), 'models', 'urdf','d-grape.urdf.xacro')
    
    ## Nodes
    nodes = GroupAction(
        actions=[
            PushRosNamespace(condition=IfCondition(LaunchConfiguration('use_namespace')), 
                            namespace=LaunchConfiguration('robot_name')),

            # Controllers spawner
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["diff_drive_controller", "joint_state_broadcaster"],
                parameters=[{
                    'use_sim_time' : LaunchConfiguration('sim')
                }],
                remappings=[('/tf', 'tf'), 
                            ('/tf_static', 'tf_static')],
            ),
        ]
    )
 
    ## Launch description
    return LaunchDescription([
        
        # Arguments
        robot_name_arg,
        namespace_arg,

        # Nodes
        nodes,

    ])