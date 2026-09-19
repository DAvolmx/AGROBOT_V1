"""
Modo NAVEGACIÓN — misión autónoma normal, con un mapa ya guardado.

  ros2 launch agrobot_navigation agrobot_navigation.launch.py

Sube: sllidar_ros2 + robot_localization (EKF) + slam_toolbox en modo
localization (contra maps/agrobot_map.yaml, generado en el modo mapeo) +
el bringup estándar de Nav2 (controller, planner, behavior, bt_navigator,
costmaps, velocity_smoother — parámetros en config/nav2_params.yaml).

jetson_server.py (jetson/backend/) debe estar corriendo aparte, en el mismo
entorno con ROS 2 activado (`source /opt/ros/<distro>/setup.bash`) para
que `rclpy` esté disponible — ese proceso es el que suscribe /cmd_vel y
lo traduce al UDP binario hacia el ESP32-C3, y el que expone
POST /api/nav/set_goal desde el dashboard.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


# ---------------------------------------------------------------------------
# TF ESTÁTICOS DEL ROBOT — sin esto slam_toolbox NO arranca.
#
# slam_toolbox necesita poder transformar cada /scan al `base_frame`
# (base_link). Esa transformación no la publica nadie automáticamente: el
# driver del LIDAR solo marca sus scans con `frame_id: laser`, y
# robot_localization publica odom->base_link, no base_link->laser. Sin este
# puente el árbol TF queda partido en dos, slam_toolbox descarta TODOS los
# scans en silencio (ni siquiera loguea error por defecto) y /map nunca
# llega a publicarse — exactamente el síntoma de "el SLAM no funciona".
#
# Las medidas son del montaje real: ajustar si se mueve el LIDAR o la IMU.
# Orden de argumentos: x y z yaw pitch roll frame_padre frame_hijo
LIDAR_XYZ = ("0.0", "0.0", "0.18")   # LIDAR 18 cm por encima del centro de base_link
LIDAR_RPY = ("0.0", "0.0", "0.0")    # yaw pitch roll — 0 = mirando al frente del robot
IMU_XYZ = ("0.0", "0.0", "0.05")
IMU_RPY = ("0.0", "0.0", "0.0")


def _tf_estaticos():
    """base_link -> laser y base_link -> imu_link."""
    return [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="tf_base_to_laser",
            arguments=["--x", LIDAR_XYZ[0], "--y", LIDAR_XYZ[1], "--z", LIDAR_XYZ[2],
                       "--yaw", LIDAR_RPY[0], "--pitch", LIDAR_RPY[1], "--roll", LIDAR_RPY[2],
                       "--frame-id", "base_link", "--child-frame-id", "laser"],
            output="log",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="tf_base_to_imu",
            arguments=["--x", IMU_XYZ[0], "--y", IMU_XYZ[1], "--z", IMU_XYZ[2],
                       "--yaw", IMU_RPY[0], "--pitch", IMU_RPY[1], "--roll", IMU_RPY[2],
                       "--frame-id", "base_link", "--child-frame-id", "imu_link"],
            output="log",
        ),
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory("agrobot_navigation")
    ekf_yaml = os.path.join(pkg_share, "config", "ekf.yaml")
    slam_yaml = os.path.join(pkg_share, "config", "slam_toolbox_localization.yaml")
    nav2_yaml = os.path.join(pkg_share, "config", "nav2_params.yaml")

    # Nodo directo y no el sllidar_c1_launch.py del paquete: ese no deja
    # activar `respawn`. El driver muere al arrancar con 80008004 (timeout
    # del RPLIDAR) en ~1 de cada 4 arranques (4 de 15 en logs/ros.log al
    # 2026-09-16), y como el resto del launch sigue vivo systemd no lo
    # relanza: el robot quedaba sin /scan y sin mapa hasta reiniciar a mano.
    # Parámetros idénticos a los de sllidar_c1_launch.py.
    lidar_launch = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="sllidar_node",
        parameters=[{
            "channel_type": "serial",
            "serial_port": "/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_5e723775cd73ef11b311cf8c8fcc3fa0-if00-port0",
            "serial_baudrate": 460800,
            "frame_id": "laser",
            "inverted": False,
            "angle_compensate": True,
            "scan_mode": "Standard",
        }],
        output="screen",
        respawn=True,
        respawn_delay=3.0,   # el puerto serie necesita soltarse antes de reabrirlo
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_yaml],
    )

    slam_node = Node(
        package="slam_toolbox",
        executable="localization_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_yaml],
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("nav2_bringup"), "launch", "navigation_launch.py")
        ),
        launch_arguments={"params_file": nav2_yaml, "use_sim_time": "false"}.items(),
    )

    # slam_toolbox en Jazzy es un LIFECYCLE node: al arrancar se queda en
    # estado `unconfigured` y NUNCA se suscribe a /scan por su cuenta. Sin
    # alguien que lo transicione a `active`, el nodo aparece vivo en
    # `ros2 node list`, no imprime un solo error, y /map jamás se publica —
    # el segundo motivo por el que "el SLAM no funcionaba".
    # nav2_lifecycle_manager lo configura y activa solo al arrancar.
    slam_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        output="screen",
        parameters=[{
            "autostart": True,
            "node_names": ["slam_toolbox"],
            "bond_timeout": 0.0,
        }],
    )

    # --- Odometría por casamiento de scans (rf2o) ---------------------------
    # Mismo montaje que en agrobot_mapping.launch.py; ver allí el detalle de
    # por qué `publish_tf` va en false y por qué `init_pose_from_topic` tiene
    # que quedar VACÍO (con su valor por defecto el nodo no publica nunca).
    filtro_scan = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        name="filtro_scan_odom",
        output="log",
        parameters=[{
            "filter1.name": "rango",
            "filter1.type": "laser_filters/LaserScanRangeFilter",
            "filter1.params.use_message_range_limits": False,
            "filter1.params.lower_threshold": 0.15,
            "filter1.params.upper_threshold": 6.0,
            "filter1.params.lower_replacement_value": float("-inf"),
            "filter1.params.upper_replacement_value": float("inf"),
        }],
        remappings=[("scan", "/scan"), ("scan_filtered", "/scan_odom")],
    )

    rf2o_node = Node(
        package="rf2o_laser_odometry",
        executable="rf2o_laser_odometry_node",
        name="rf2o_laser_odometry",
        output="screen",
        parameters=[{
            "laser_scan_topic": "/scan_odom",
            "odom_topic": "/odom_rf2o",
            "publish_tf": False,
            "base_frame_id": "base_link",
            "odom_frame_id": "odom",
            "init_pose_from_topic": "",
            "freq": 10.0,
        }],
    )

    return LaunchDescription([*_tf_estaticos(), lidar_launch,
                              filtro_scan, rf2o_node,
                              ekf_node, slam_node, slam_lifecycle,
                              nav2_launch])
