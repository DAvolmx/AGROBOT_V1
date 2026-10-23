"""SLAM del robot reproducido SIN el robot (2026-10-22): mismos nodos que agrobot_mapping.launch.py (filtros, rf2o, EKF, slam_toolbox),
alimentados por simulador_pasivo/slam_ros/reproducir.py con los scans y el IMU grabados. Corre con reloj simulado y en OTRO ROS_DOMAIN_ID
(lo pone simulador_pasivo/simular_slam.py) para no mezclarse con el ROS vivo del robot.

Argumentos:
  slam_yaml   archivo de parámetros de slam_toolbox (por defecto el del robot)
  caja        true = filtro de caja que quita el propio chasis del scan (la mejora que se evalúa)
  laser_girado true (por defecto) = como en el robot real: láser montado a 180° (TF yaw=π) y scans en el marco del láser.
              Con false (lo que se usó hasta el 2026-10-23) los scans iban ya en el marco del robot y eso ESCONDÍA el vx invertido de rf2o.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Huella del robot vista desde base_link (= el LIDAR en x,y): 18,5 cm delante, 30,5 cm detrás, 38 cm de ancho; + margen para el soporte
# de la cámara/servo que aparece a ~15-20 cm a -35° en todos los scans grabados.
CAJA = {"min_x": -0.34, "max_x": 0.22, "min_y": -0.23, "max_y": 0.23, "min_z": -1.0, "max_z": 1.0}


def _filtros(caja: bool, rango_max: float, mediana: bool):
    p = {"filter1.name": "rango", "filter1.type": "laser_filters/LaserScanRangeFilter",
         "filter1.params.use_message_range_limits": False, "filter1.params.lower_threshold": 0.18,
         "filter1.params.upper_threshold": rango_max, "filter1.params.lower_replacement_value": float("-inf"),
         "filter1.params.upper_replacement_value": float("inf"), "use_sim_time": True}
    n = 2
    if caja:
        p.update({f"filter{n}.name": "chasis", f"filter{n}.type": "laser_filters/LaserScanBoxFilter",
                  f"filter{n}.params.box_frame": "base_link", f"filter{n}.params.invert": False,
                  **{f"filter{n}.params.{k}": v for k, v in CAJA.items()}})
        n += 1
    if mediana:
        p.update({f"filter{n}.name": "mediana_espacial", f"filter{n}.type": "laser_filters/LaserScanMedianSpatialFilter",
                  f"filter{n}.params.window_size": 3})
    return p


def _nodos(context):
    caja = LaunchConfiguration("caja").perform(context).lower() == "true"
    slam_yaml = LaunchConfiguration("slam_yaml").perform(context)
    cfg = os.path.join(get_package_share_directory("agrobot_navigation"), "config")
    sim = {"use_sim_time": True}
    girado = LaunchConfiguration("laser_girado").perform(context).lower() == "true"
    tf = lambda nombre, hijo, z, yaw="0": Node(package="tf2_ros", executable="static_transform_publisher", name=nombre, output="log",
                                               arguments=["--z", z, "--yaw", yaw, "--frame-id", "base_link", "--child-frame-id", hijo],
                                               parameters=[sim])
    return [
        tf("tf_base_to_laser", "laser", "0.18", "3.141592653589793" if girado else "0"),
        tf("tf_base_to_imu", "imu_link", "0.05"),
        Node(package="laser_filters", executable="scan_to_scan_filter_chain", name="filtro_scan_odom", output="log",
             parameters=[_filtros(caja, 6.0, False)], remappings=[("scan", "/scan"), ("scan_filtered", "/scan_odom")]),
        Node(package="laser_filters", executable="scan_to_scan_filter_chain", name="filtro_scan_slam", output="log",
             parameters=[_filtros(caja, 8.0, True)], remappings=[("scan", "/scan"), ("scan_filtered", "/scan_slam")]),
        Node(package="rf2o_laser_odometry", executable="rf2o_laser_odometry_node", name="rf2o_laser_odometry", output="log",
             ros_arguments=["--log-level", "rf2o_laser_odometry:=error"],
             parameters=[{"laser_scan_topic": "/scan_odom", "odom_topic": "/odom_rf2o", "publish_tf": False, "base_frame_id": "base_link",
                          "odom_frame_id": "odom", "init_pose_from_topic": "", "freq": 10.0, "use_sim_time": True}]),
        Node(package="robot_localization", executable="ekf_node", name="ekf_filter_node", output="log",
             parameters=[os.path.join(cfg, "ekf.yaml"), sim]),
        Node(package="slam_toolbox", executable="async_slam_toolbox_node", name="slam_toolbox", output="log",
             parameters=[slam_yaml, sim]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_slam", output="log",
             parameters=[{"autostart": True, "node_names": ["slam_toolbox"], "bond_timeout": 0.0, "use_sim_time": True}]),
    ]


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory("agrobot_navigation"), "config")
    return LaunchDescription([
        DeclareLaunchArgument("slam_yaml", default_value=os.path.join(cfg, "slam_toolbox_mapping.yaml")),
        DeclareLaunchArgument("caja", default_value="false"),
        DeclareLaunchArgument("laser_girado", default_value="true"),
        OpaqueFunction(function=_nodos),
    ])
