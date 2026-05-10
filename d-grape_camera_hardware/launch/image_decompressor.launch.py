"""
image_decompressor.launch.py

Запускается НА НОУТБУКЕ.
Подписывается на CompressedImage топики от stereo_cameras_node (по WiFi)
и переиздаёт их как обычный sensor_msgs/Image для любых downstream-нод.

Входные топики (CompressedImage):
  /front_cams/f_left_camera/image/compressed
  /front_cams/f_right_camera/image/compressed
  /left_cams/l_left_camera/image/compressed
  /left_cams/l_right_camera/image/compressed
  /right_cams/r_left_camera/image/compressed
  /right_cams/r_right_camera/image/compressed

Выходные топики (Image):
  /front_cams/f_left_camera/image
  …и т.д. (убирается суффикс /compressed)

Аргументы запуска:
  qos_reliability — best_effort|reliable (default: best_effort)
  qos_depth       — глубина очереди     (default: 1)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# Список топиков по умолчанию — можно переопределить через параметр ноды
DEFAULT_TOPICS = [
    '/front_cams/f_left_camera/image/compressed',
    '/front_cams/f_right_camera/image/compressed',
    '/left_cams/l_left_camera/image/compressed',
    '/left_cams/l_right_camera/image/compressed',
    '/right_cams/r_left_camera/image/compressed',
    '/right_cams/r_right_camera/image/compressed',
]


def generate_launch_description():
    # ── аргументы командной строки ──────────────────────────────────────────
    rel_arg = DeclareLaunchArgument(
        'qos_reliability', default_value='best_effort',
        description='QoS reliability: best_effort | reliable')
    depth_arg = DeclareLaunchArgument(
        'qos_depth', default_value='1',
        description='QoS history depth')

    rel   = ParameterValue(LaunchConfiguration('qos_reliability'), value_type=str)
    depth = ParameterValue(LaunchConfiguration('qos_depth'),       value_type=int)

    # ── нода декомпрессора ──────────────────────────────────────────────────
    decompressor_node = Node(
        package='d-grape_camera_hardware',
        executable='image_decompressor_node',
        name='image_decompressor',
        output='screen',
        parameters=[{
            'topics':           DEFAULT_TOPICS,
            'qos_reliability':  rel,
            'qos_depth':        depth,
        }],
    )

    return LaunchDescription([
        rel_arg,
        depth_arg,
        decompressor_node,
    ])
