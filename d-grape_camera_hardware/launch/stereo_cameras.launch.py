"""
stereo_cameras.launch.py

Запускает ОДНУ ноду stereo_cameras_node, которая управляет всеми
тремя стерео-парами через единый таймер.

Публикуемые топики (до 12 Image + до 6 CompressedImage + 6 CameraInfo):
  /front_cams/f_left_camera/image         /front_cams/f_left_camera/camera_info
  /front_cams/f_right_camera/image        /front_cams/f_right_camera/camera_info
  /left_cams/l_left_camera/image          /left_cams/l_left_camera/camera_info
  /left_cams/l_right_camera/image         /left_cams/l_right_camera/camera_info
  /right_cams/r_left_camera/image         /right_cams/r_left_camera/camera_info
  /right_cams/r_right_camera/image        /right_cams/r_right_camera/camera_info
  (+ /…/image/compressed при publish_compressed:=true)

Аргументы запуска:
  camera_fps          — частота кадров (по умолчанию 30.0)
  frame_width         — ширина SBS кадра (по умолчанию 1280)
  frame_height        — высота кадра (по умолчанию 480)
  baseline_m          — база стерео в метрах (по умолчанию 0.178)
  publish_raw         — публиковать sensor_msgs/Image (по умолчанию true)
  publish_compressed  — публиковать CompressedImage (по умолчанию false)
  jpeg_quality        — качество JPEG 1–100 (по умолчанию 80)
  img_reliability     — QoS для Image/CompressedImage: best_effort|reliable
  img_depth           — глубина очереди Image/CompressedImage (по умолчанию 1)
  info_reliability    — QoS для CameraInfo: best_effort|reliable
  info_depth          — глубина очереди CameraInfo (по умолчанию 10)
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

CONFIG_ROOT = get_package_share_directory('d-grape_camera_hardware') + '/config'


def generate_launch_description():
    # ── аргументы командной строки ──────────────────────────────────────────
    fps_arg      = DeclareLaunchArgument(
        'camera_fps',   default_value='30.0',
        description='FPS для всех камер')
    width_arg    = DeclareLaunchArgument(
        'frame_width',  default_value='1280',
        description='Ширина SBS кадра (оба глаза вместе)')
    height_arg   = DeclareLaunchArgument(
        'frame_height', default_value='480',
        description='Высота кадра')
    baseline_arg = DeclareLaunchArgument(
        'baseline_m',   default_value='0.178',
        description='База стерео пары в метрах')

    raw_arg = DeclareLaunchArgument(
        'publish_raw',        default_value='true',
        description='Публиковать sensor_msgs/Image (raw)')
    compressed_arg = DeclareLaunchArgument(
        'publish_compressed', default_value='false',
        description='Публиковать sensor_msgs/CompressedImage (JPEG)')
    quality_arg = DeclareLaunchArgument(
        'jpeg_quality',       default_value='80',
        description='Качество JPEG 1–100 (только если publish_compressed=true)')

    img_rel_arg = DeclareLaunchArgument(
        'img_reliability',    default_value='best_effort',
        description='QoS reliability для Image/CompressedImage: best_effort|reliable')
    img_depth_arg = DeclareLaunchArgument(
        'img_depth',          default_value='1',
        description='QoS history depth для Image/CompressedImage')
    info_rel_arg = DeclareLaunchArgument(
        'info_reliability',   default_value='reliable',
        description='QoS reliability для CameraInfo: best_effort|reliable')
    info_depth_arg = DeclareLaunchArgument(
        'info_depth',         default_value='10',
        description='QoS history depth для CameraInfo')

    # ── подстановки ─────────────────────────────────────────────────────────
    fps      = ParameterValue(LaunchConfiguration('camera_fps'),        value_type=float)
    width    = ParameterValue(LaunchConfiguration('frame_width'),       value_type=int)
    height   = ParameterValue(LaunchConfiguration('frame_height'),      value_type=int)
    baseline = ParameterValue(LaunchConfiguration('baseline_m'),        value_type=float)

    raw        = ParameterValue(LaunchConfiguration('publish_raw'),        value_type=bool)
    compressed = ParameterValue(LaunchConfiguration('publish_compressed'), value_type=bool)
    quality    = ParameterValue(LaunchConfiguration('jpeg_quality'),       value_type=int)

    img_rel   = ParameterValue(LaunchConfiguration('img_reliability'),  value_type=str)
    img_depth = ParameterValue(LaunchConfiguration('img_depth'),        value_type=int)
    info_rel  = ParameterValue(LaunchConfiguration('info_reliability'), value_type=str)
    info_depth = ParameterValue(LaunchConfiguration('info_depth'),      value_type=int)

    # ── единственная нода ────────────────────────────────────────────────────
    cameras_node = Node(
        package='d-grape_camera_hardware',
        executable='stereo_camera_all_cams_qos_compressed',
        name='stereo_camera_all_cams',
        output='screen',
        parameters=[{
            'camera_fps':           fps,
            'frame_width':          width,
            'frame_height':         height,
            'baseline_m':           baseline,
            'config_root':          CONFIG_ROOT,
            'reconnect_sec':        2.0,
            'publish_raw':          raw,
            'publish_compressed':   compressed,
            'jpeg_quality':         quality,
            'img_reliability':      img_rel,
            'img_depth':            img_depth,
            'info_reliability':     info_rel,
            'info_depth':           info_depth,
        }],
    )

    return LaunchDescription([
        fps_arg,
        width_arg,
        height_arg,
        baseline_arg,
        raw_arg,
        compressed_arg,
        quality_arg,
        img_rel_arg,
        img_depth_arg,
        info_rel_arg,
        info_depth_arg,
        cameras_node,
    ])
