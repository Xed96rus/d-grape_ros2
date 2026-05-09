"""
Launch-файл: запускает три экземпляра stereo_camera_node (C++).

Каждая нода открывает ОДИН USB-девайс (/dev/NAME_cams),
захватывает MJPG кадр 1280×480, делит пополам и публикует:

  namespace left_cams   (камера 1, /dev/left_cams):
    /left_cams/l_left_camera/image          /left_cams/l_left_camera/camera_info
    /left_cams/l_right_camera/image         /left_cams/l_right_camera/camera_info

  namespace right_cams  (камера 2, /dev/right_cams):
    /right_cams/r_left_camera/image         /right_cams/r_left_camera/camera_info
    /right_cams/r_right_camera/image        /right_cams/r_right_camera/camera_info

Итого 12 топиков — 4 на камеру (image + camera_info для left и right).
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# =============================================================================
# Привязка физических портов к namespace-ам
# Симлинки создаются udev-правилами из /etc/udev/rules.d/99-stereo-camera.rules
# =============================================================================
CONFIG_ROOT = get_package_share_directory('d-grape_camera_hardware') + '/config'
CAMERA_CONFIG = [
    # (namespace,    device_path,           prefix, left_yaml,      right_yaml)
    ('left_cams',  '/dev/stereo_cam1',     'l',     'left_cal/left.yaml',   'left_cal/right.yaml'),
    ('right_cams', '/dev/stereo_cam2',     'r',     'right_cal/left.yaml',  'right_cal/right.yaml'),
]
# =============================================================================


def _make_stereo_node(namespace, device_path, prefix, left_yaml, right_yaml, fps, width, height) -> Node:
    p = prefix
    return Node(
        package='d-grape_camera_hardware',
        executable='stereo_camera_node',
        name=f'stereo_camera_node_{namespace}',
        namespace=namespace,
        output='screen',
        parameters=[{
            'device_path':         device_path,
            'camera_fps':          fps,
            'frame_width':         width,
            'frame_height':        height,
            'frame_id_left':       f'{p}_left_camera',
            'frame_id_right':      f'{p}_right_camera',
            'left_camera_info_url':  f'{CONFIG_ROOT}/{left_yaml}',
            'right_camera_info_url': f'{CONFIG_ROOT}/{right_yaml}',
        }],
        remappings=[
            ('left/image_raw',    f'{p}_left_camera/image'),
            ('left/camera_info',  f'{p}_left_camera/camera_info'),
            ('right/image_raw',   f'{p}_right_camera/image'),
            ('right/camera_info', f'{p}_right_camera/camera_info'),
        ],
    )


def generate_launch_description():
    fps_arg    = DeclareLaunchArgument(
        'camera_fps',   default_value='30.0',  description='FPS для всех камер')
    width_arg  = DeclareLaunchArgument(
        'frame_width',  default_value='1280',  description='Ширина wide-кадра (side-by-side)')
    height_arg = DeclareLaunchArgument(
        'frame_height', default_value='480',   description='Высота кадра')

    fps    = ParameterValue(LaunchConfiguration('camera_fps'),   value_type=float)
    width  = ParameterValue(LaunchConfiguration('frame_width'),  value_type=int)
    height = ParameterValue(LaunchConfiguration('frame_height'), value_type=int)

    nodes = [
        _make_stereo_node(ns, dev, prefix, left_yaml, right_yaml, fps, width, height)
        for ns, dev, prefix, left_yaml, right_yaml in CAMERA_CONFIG
    ]

    return LaunchDescription([fps_arg, width_arg, height_arg, *nodes])
