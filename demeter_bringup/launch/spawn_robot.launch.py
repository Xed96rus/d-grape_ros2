import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node, PushRosNamespace
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # --- Arguments ---
    robot_name_arg = DeclareLaunchArgument(
        name='robot_name',
        default_value='demeter',
        description='Unique robot name'
    )
    use_namespace_arg = DeclareLaunchArgument(
        name='use_namespace',
        default_value='false',
        choices=['true', 'false'],
        description='Use robot name as namespace'
    )
    x_arg = DeclareLaunchArgument(name='x', default_value='0.0', description='x-position')
    y_arg = DeclareLaunchArgument(name='y', default_value='0.0', description='y-position')
    z_arg = DeclareLaunchArgument(name='z', default_value='2.0', description='z-position')

    robot_name = LaunchConfiguration('robot_name')
    use_namespace = LaunchConfiguration('use_namespace')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')

    # --- Xacro file ---
    pkg_description = get_package_share_directory('demeter_description')
    xacro_file = os.path.join(pkg_description, 'models', 'urdf', 'demeter.urdf.xacro')

    # --- Without namespace ---
    nodes_no_ns = GroupAction(
        condition=UnlessCondition(use_namespace),
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    ['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
                    ['/front_cams/f_left_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
                    ['/front_cams/f_left_camera/image@sensor_msgs/msg/Image[gz.msgs.Image'],
                    ['/front_cams/f_right_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
                    ['/front_cams/f_right_camera/image@sensor_msgs/msg/Image[gz.msgs.Image'],

                    ['/left_cams/l_left_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
                    ['/left_cams/l_left_camera/image@sensor_msgs/msg/Image[gz.msgs.Image'],
                    ['/left_cams/l_right_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
                    ['/left_cams/l_right_camera/image@sensor_msgs/msg/Image[gz.msgs.Image'],

                    ['/right_cams/r_left_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
                    ['/right_cams/r_left_camera/image@sensor_msgs/msg/Image[gz.msgs.Image'],
                    ['/right_cams/r_right_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
                    ['/right_cams/r_right_camera/image@sensor_msgs/msg/Image[gz.msgs.Image'],

                    ['/navsat/fix@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat'],
                    ['/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU']
                ],
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                }]
            ),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                parameters=[{
                    'use_sim_time' : True,
                    'robot_description': ParameterValue(
                        Command([
                            'xacro ', xacro_file,
                            ' sim:=true',
                            ' robot_name:=', robot_name,
                            ' namespace:=',
                            ' frame_prefix:='
                        ]), value_type=str
                    )
                }],
                output='screen'
            ),
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', robot_name,
                    '-topic', '/robot_description',
                    '-x', x,
                    '-y', y,
                    '-z', z
                ],
                parameters=[{
                    'use_sim_time' : True
                }],
                output='screen'
            ),
        ]
    )

    # --- With namespace ---
    nodes_with_ns = GroupAction(
        condition=IfCondition(use_namespace),
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    ['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
                    [robot_name, '/front_cams/f_left_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
                    [robot_name, '/front_cams/f_left_camera/image@sensor_msgs/msg/Image[gz.msgs.Image'],
                    [robot_name, '/front_cams/f_right_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
                    [robot_name, '/front_cams/f_right_camera/image@sensor_msgs/msg/Image[gz.msgs.Image'],

                    [robot_name, '/left_cams/l_left_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
                    [robot_name, '/left_cams/l_left_camera/image@sensor_msgs/msg/Image[gz.msgs.Image'],
                    [robot_name, '/left_cams/l_right_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
                    [robot_name, '/left_cams/l_right_camera/image@sensor_msgs/msg/Image[gz.msgs.Image'],

                    [robot_name, '/right_cams/r_left_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
                    [robot_name, '/right_cams/r_left_camera/image@sensor_msgs/msg/Image[gz.msgs.Image'],
                    [robot_name, '/right_cams/r_right_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
                    [robot_name, '/right_cams/r_right_camera/image@sensor_msgs/msg/Image[gz.msgs.Image'],
                    
                    [robot_name, '/navsat/fix@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat'],
                    [robot_name, '/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU']
                ],
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                }]
            ),
            PushRosNamespace(robot_name),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                parameters=[{
                    'use_sim_time' : True,
                    'frame_prefix': [robot_name, '/'],
                    'robot_description': ParameterValue(
                        Command([
                            'xacro ', xacro_file,
                            ' sim:=true',
                            ' robot_name:=', robot_name,
                            ' namespace:=', robot_name,
                            ' frame_prefix:=', robot_name, '/'
                        ]), value_type=str
                    )
                }],
                remappings=[('/tf', 'tf'), 
                            ('/tf_static', 'tf_static')],
                output='screen'
            ),
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', robot_name,
                    '-topic', ('/', robot_name, '/robot_description'),
                    '-x', x,
                    '-y', y,
                    '-z', z
                ],
                parameters=[{
                    'use_sim_time' : True
                }],
                output='screen'
            ),
        ]
    )

    return LaunchDescription([
        robot_name_arg,
        use_namespace_arg,
        x_arg,
        y_arg,
        z_arg,
        nodes_no_ns,
        nodes_with_ns,
    ])