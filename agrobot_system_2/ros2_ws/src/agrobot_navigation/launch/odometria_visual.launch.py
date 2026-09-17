"""
Odometría visual (rtabmap rgbd_odometry) — preparada, APAGADA por defecto.
=========================================================================

Se incluye desde agrobot_mapping.launch.py y solo hace algo con
`odom_visual:=true`. Para activarla en el robot, en /etc/default/agrobot:

    AGROBOT_LAUNCH_ARGS="odom_visual:=true camara:=realsense"
    sudo systemctl restart agrobot-ros

Levanta el driver de la cámara, la TF de montaje y rgbd_odometry, que
publica /odom_visual. El EKF (ekf.yaml, `odom2`) ya está suscrito a ese
tópico: mientras nadie lo publique no cambia nada.

Presets de cámara: los tópicos son los que publican por defecto los drivers
oficiales. NO están comprobados con la cámara real (no la había al
escribirlo): al conectarla, verificar con `ros2 topic list` y corregir aquí.

Al montar la cámara hay que medir su posición y rellenar CAM_XYZ / CAM_RPY:
una TF de montaje mal puesta convierte los giros en traslaciones falsas.
"""
import os

from ament_index_python.packages import (PackageNotFoundError,
                                         get_package_share_directory)
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ⚠️ SIN MEDIR. Posición del centro óptico respecto de base_link (centro del
# robot a ras del chasis): x hacia delante, z hacia arriba, en metros.
CAM_XYZ = ("0.20", "0.0", "0.25")
CAM_RPY = ("0.0", "0.0", "0.0")      # yaw pitch roll; 0 = mirando al frente

PRESETS = {
    # Intel RealSense D435/D435i — paquete realsense2_camera (apt:
    # ros-jazzy-realsense2-camera). Profundidad alineada al color.
    "realsense": {
        "paquete": "realsense2_camera",
        "launch": "rs_launch.py",
        "args": {
            "enable_depth": "true",
            "align_depth.enable": "true",
            "enable_gyro": "false",
            "enable_accel": "false",
            "pointcloud.enable": "false",
            "rgb_camera.color_profile": "640x480x15",
            "depth_module.depth_profile": "640x480x15",
        },
        "marco_camara": "camera_link",
        "rgb": "/camera/camera/color/image_raw",
        "profundidad": "/camera/camera/aligned_depth_to_color/image_raw",
        "info": "/camera/camera/color/camera_info",
    },
    # Luxonis OAK-D / OAK-D Lite — paquete depthai_ros_driver (apt:
    # ros-jazzy-depthai-ros-driver). Pipeline RGBD con estéreo alineado a RGB.
    "oak": {
        "paquete": "depthai_ros_driver",
        "launch": "camera.launch.py",
        "args": {},
        "marco_camara": "oak-d-base-frame",
        "rgb": "/oak/rgb/image_raw",
        "profundidad": "/oak/stereo/image_raw",
        "info": "/oak/rgb/camera_info",
    },
}


def _construir(context):
    if LaunchConfiguration("odom_visual").perform(context).lower() != "true":
        return []

    nombre = LaunchConfiguration("camara").perform(context)
    preset = PRESETS.get(nombre)
    if preset is None:
        return [LogInfo(msg=f"[odom_visual] cámara desconocida '{nombre}'. "
                            f"Opciones: {', '.join(PRESETS)}. No se lanza la odometría visual.")]

    # El driver se resuelve acá, en tiempo de lanzamiento: si la cámara no
    # está instalada, el resto del robot tiene que seguir arrancando.
    try:
        share = get_package_share_directory(preset["paquete"])
    except PackageNotFoundError:
        return [LogInfo(msg=f"[odom_visual] falta el driver '{preset['paquete']}'. "
                            f"Instalar con: sudo apt install ros-jazzy-"
                            f"{preset['paquete'].replace('_', '-')}. "
                            f"El robot sigue sin odometría visual.")]

    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", preset["launch"])),
        launch_arguments=preset["args"].items(),
    )
    tf_montaje = Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="tf_base_to_camara_odom",
        arguments=["--x", CAM_XYZ[0], "--y", CAM_XYZ[1], "--z", CAM_XYZ[2],
                   "--yaw", CAM_RPY[0], "--pitch", CAM_RPY[1], "--roll", CAM_RPY[2],
                   "--frame-id", "base_link", "--child-frame-id", preset["marco_camara"]],
        output="log",
    )
    config = os.path.join(get_package_share_directory("agrobot_navigation"),
                          "config", "odometria_visual.yaml")
    odometria = Node(
        package="rtabmap_odom", executable="rgbd_odometry", name="rgbd_odometry",
        output="screen", parameters=[config],
        remappings=[("rgb/image", preset["rgb"]),
                    ("depth/image", preset["profundidad"]),
                    ("rgb/camera_info", preset["info"]),
                    ("odom", "/odom_visual")],
        respawn=True, respawn_delay=3.0,
    )
    return [LogInfo(msg=f"[odom_visual] activa con cámara '{nombre}'"),
            driver, tf_montaje, odometria]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("odom_visual", default_value="false",
                              description="true = odometría visual con cámara de profundidad"),
        DeclareLaunchArgument("camara", default_value="realsense",
                              description=f"modelo de cámara: {', '.join(PRESETS)}"),
        OpaqueFunction(function=_construir),
    ])
