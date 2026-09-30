"""
Modo MAPEO — primera pasada por un terreno nuevo.

  ros2 launch agrobot_navigation agrobot_mapping.launch.py

Es el launch que usa systemd (agrobot-ros.service) en el día a día.

Sube: sllidar_ros2 (driver real del RPLIDAR C1) + rf2o + robot_localization
(EKF) + slam_toolbox en modo mapping + Nav2 (desde 2026-09-17), así la
patrulla autónoma navega sobre el mapa mientras este sigue creciendo. Los
mapas se guardan y reusan con nombre desde el dashboard (backend/mapas.py).

Opcional: odometría visual con `odom_visual:=true camara:=realsense|oak`
(ver odometria_visual.launch.py).
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
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
# yaw = pi: con el C1 montado segun la flecha de la carcasa, el 0 grados que
# publica sllidar_ros2 apunta HACIA ATRAS del robot (medido 2026-09-19: una
# persona 90 cm detras aparecia en 0 grados). Mantener igual que
# LIDAR_YAW_DEG de backend/ros_bridge.py.
LIDAR_RPY = ("3.141592653589793", "0.0", "0.0")    # yaw pitch roll
IMU_XYZ = ("0.0", "0.0", "0.05")

# Puerto del RPLIDAR C1: lo elige scripts/elegir_puerto_lidar.sh en CADA
# arranque del driver (también en los respawn): el adaptador USB CP2102N si
# está enchufado, si no la UART del conector (pines 8/10 -> /dev/ttyTHS1).
# `lidar_puerto:=/dev/...` lo fuerza. La misma regla usa el vigía del
# backend (jetson_server._puerto_lidar).
LIDAR_PUERTO = "auto"
ELEGIR_PUERTO = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                             "..", "scripts", "elegir_puerto_lidar.sh")
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
    slam_yaml = os.path.join(pkg_share, "config", "slam_toolbox_mapping.yaml")

    # Nodo directo y no el sllidar_c1_launch.py del paquete: ese no deja
    # activar `respawn`. El driver muere al arrancar con 80008004 (timeout
    # del RPLIDAR) en ~1 de cada 4 arranques (4 de 15 en logs/ros.log al
    # 2026-09-16), y como el resto del launch sigue vivo systemd no lo
    # relanza: el robot quedaba sin /scan y sin mapa hasta reiniciar a mano.
    # Parámetros idénticos a los de sllidar_c1_launch.py.
    # Puerto por número de serie y NO /dev/ttyUSB0: el 2026-09-19 el LIDAR
    # se reenumeró como ttyUSB1 tras un corte USB, el driver no lo encontró,
    # y sin scans el freno por LIDAR bloqueó adelante/atrás (solo giraba).
    lidar_launch = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="sllidar_node",
        prefix=ELEGIR_PUERTO,
        additional_env={"AGROBOT_LIDAR_PUERTO": LaunchConfiguration("lidar_puerto")},
        parameters=[{
            "channel_type": "serial",
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

    # --- Odometría por casamiento de scans (rf2o) ---------------------------
    # La ÚNICA medida real de traslación del robot. No hay encoders, y hasta
    # ahora `/wheel_odom` era la velocidad COMANDADA, no la real: sobre tierra
    # y con un skid-steer que patina, eso es una suposición, no una medida.
    #
    # `publish_tf` va en FALSE a propósito. Quien publica odom -> base_link es
    # el EKF; si rf2o lo publicara también, habría dos nodos escribiendo la
    # misma transformación y TF2 se queda con la última que llegue, que va
    # cambiando. El sintoma es un robot que "tiembla" en RViz sin causa
    # aparente y cuesta mucho de diagnosticar.
    #
    # `init_pose_from_topic` va VACÍO, y esto no es opcional: el valor por
    # defecto del paquete es "/base_pose_ground_truth", un topic de simulación
    # que aquí no publica nadie. Con el valor por defecto el nodo se queda
    # esperando esa pose para siempre y NO publica odometría nunca, sin dar
    # ningún error. Está comprobado en su propio código (CLaserOdometry2DNode
    # .cpp, línea 53: `if (init_pose_from_topic != "")`).
    #
    # `freq` a 10 Hz, que es la tasa real del RPLIDAR C1. Pedir más solo hace
    # que el nodo reprocese scans repetidos.
    rf2o_node = Node(
        package="rf2o_laser_odometry",
        executable="rf2o_laser_odometry_node",
        name="rf2o_laser_odometry",
        output="screen",
        # Sin LIDAR escribe "Waiting for laser_scans" 10 veces por segundo:
        # el 2026-09-28 eso llevó logs/ros.log a 539 MB. Solo errores.
        ros_arguments=["--log-level", "rf2o_laser_odometry:=error"],
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

    # Recorte de alcance ANTES de la odometría. El chasis lleva suspensión de
    # recorrido largo y el LIDAR va a ~30 cm del suelo: con solo 2 grados de
    # cabeceo el haz ya toca el suelo a 8.6 m. Ese suelo entra y sale del scan
    # con cada bache, y el casamiento lo interpreta como traslación que no ha
    # ocurrido — justo el error que rf2o viene a eliminar.
    #
    # Quedarse con los primeros 6 m corta esa fuente de ruido de raíz. El
    # /scan completo sigue intacto para slam_toolbox y para los costmaps: el
    # recorte solo alimenta a la odometría.
    filtro_scan = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        name="filtro_scan_odom",
        output="log",
        parameters=[{
            "filter1.name": "rango",
            "filter1.type": "laser_filters/LaserScanRangeFilter",
            "filter1.params.use_message_range_limits": False,
            # 0.18 m descarta los retornos del propio chasis y de los cables.
            # Con 0.15 se colaba uno a 15,5 cm (mástil de la cámara, medido
            # en 100 scans el 2026-09-30); 0.175 m es el radio inscrito de la
            # carcasa, así que nada real puede estar más cerca del LIDAR.
            "filter1.params.lower_threshold": 0.18,
            "filter1.params.upper_threshold": 6.0,
            # Lo que cae fuera se marca como infinito, no como NaN: infinito
            # significa "no hay nada ahi", que es la lectura correcta para un
            # haz recortado. NaN significa "medida invalida" y no es lo mismo.
            "filter1.params.lower_replacement_value": float("-inf"),
            "filter1.params.upper_replacement_value": float("inf"),
        }],
        remappings=[("scan", "/scan"), ("scan_filtered", "/scan_odom")],
    )

    # Filtro de scans para SLAM — mediana espacial para suavizar paredes y
    # eliminar picos aislados de ruido antes del casamiento de scans. Limita
    # también el rango a 8 m (el C1 es fiable hasta ahí; más allá domina el
    # multipath en interiores). slam_toolbox lee /scan_slam.
    filtro_scan_slam = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        name="filtro_scan_slam",
        output="log",
        parameters=[{
            "filter1.name": "rango_slam",
            "filter1.type": "laser_filters/LaserScanRangeFilter",
            "filter1.params.use_message_range_limits": False,
            "filter1.params.lower_threshold": 0.18,
            "filter1.params.upper_threshold": 8.0,
            "filter1.params.lower_replacement_value": float("-inf"),
            "filter1.params.upper_replacement_value": float("inf"),
            "filter2.name": "mediana_espacial",
            "filter2.type": "laser_filters/LaserScanMedianSpatialFilter",
            "filter2.params.window_size": 3,
        }],
        remappings=[("scan", "/scan"), ("scan_filtered", "/scan_slam")],
    )

    slam_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_yaml],
    )

    # --- Banco de pruebas: odom estático ------------------------------------
    # El EKF NO publica odom->base_link hasta recibir su primera medición, y
    # esa medición la manda jetson_server.py (/imu/data_raw, desde el MPU6050
    # que llega por LoRa). Con el backend apagado el árbol TF queda sin ese
    # eslabón y slam_toolbox se queda mudo, igual que si faltaran los TF
    # estáticos — dos causas distintas con el MISMO síntoma, difíciles de
    # distinguir a ojo.
    #
    # Con `odom_estatico:=true` se publica un odom->base_link fijo para poder
    # verificar el resto de la cadena (LIDAR -> TF -> slam_toolbox -> /map) en
    # el banco, sin backend ni telemetría. El robot NO se puede mover en ese
    # modo: la pose queda clavada en el origen y el mapa saldría corrupto.
    # Nunca usarlo en campo.
    odom_estatico = LaunchConfiguration("odom_estatico")

    odom_falso = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_odom_estatico_solo_pruebas",
        arguments=["--frame-id", "odom", "--child-frame-id", "base_link"],
        condition=IfCondition(odom_estatico),
        output="log",
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

    # ---- Nav2 SOBRE EL SLAM EN VIVO ----
    # La patrulla autónoma manda metas a `navigate_to_pose`. Este launch no
    # traía Nav2, así que activar la patrulla no movía el robot: la meta se
    # perdía esperando un servidor que no existía.
    #
    # Nav2 corre encima de slam_toolbox en modo mapeo: el global_costmap lee
    # el /map que sigue creciendo, y el local_costmap el LIDAR en crudo.
    #
    # Solo los 4 nodos que usa el árbol de comportamiento por defecto, y no
    # el navigation_launch.py de nav2_bringup: ese arranca 10 (docking,
    # route_server, collision_monitor...) y cualquiera sin configurar tira
    # abajo el arranque de todos. Por eso Nav2 nunca llegó a activarse.
    # Tampoco velocity_smoother ni collision_monitor: el backend ya suaviza
    # (rampa) y frena por LIDAR, y así controller_server publica /cmd_vel
    # directo al puente.
    nav2_yaml = os.path.join(pkg_share, "config", "nav2_params.yaml")
    nav2_nodos = ["controller_server", "planner_server", "behavior_server", "bt_navigator"]
    # Árboles de comportamiento: hay que dar la ruta COMPLETA. Ver el aviso en
    # nav2_params.yaml sobre por qué una cadena vacía no vale.
    bt_dir = os.path.join(get_package_share_directory("nav2_bt_navigator"), "behavior_trees")
    bt_params = {
        "default_nav_to_pose_bt_xml":
            os.path.join(bt_dir, "navigate_to_pose_w_replanning_and_recovery.xml"),
        "default_nav_through_poses_bt_xml":
            os.path.join(bt_dir, "navigate_through_poses_w_replanning_and_recovery.xml"),
    }
    nav2 = [
        Node(package="nav2_controller", executable="controller_server",
             name="controller_server", output="screen", parameters=[nav2_yaml]),
        Node(package="nav2_planner", executable="planner_server",
             name="planner_server", output="screen", parameters=[nav2_yaml]),
        Node(package="nav2_behaviors", executable="behavior_server",
             name="behavior_server", output="screen", parameters=[nav2_yaml]),
        Node(package="nav2_bt_navigator", executable="bt_navigator",
             name="bt_navigator", output="screen", parameters=[nav2_yaml, bt_params]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_navigation", output="screen",
             parameters=[{"autostart": True, "node_names": nav2_nodos,
                          "bond_timeout": 0.0}]),
    ]

    # Odometría visual: apagada salvo `odom_visual:=true` (ver ese launch).
    odometria_visual = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "odometria_visual.launch.py")))

    return LaunchDescription([
        DeclareLaunchArgument(
            "lidar_puerto", default_value=LIDAR_PUERTO,
            description="Puerto serie del RPLIDAR C1; 'auto' = USB si está, si no la UART."),
        DeclareLaunchArgument(
            "odom_estatico", default_value="false",
            description="true = publica un odom->base_link fijo para probar el "
                        "SLAM en el banco sin jetson_server.py. El robot no se "
                        "puede mover en este modo."),
        *_tf_estaticos(),
        odom_falso,
        lidar_launch,
        filtro_scan,       # /scan -> /scan_odom, recortado a 6 m
        filtro_scan_slam,  # /scan -> /scan_slam, filtrado para SLAM
        rf2o_node,         # /scan_odom -> /odom_rf2o
        ekf_node,
        slam_node,
        slam_lifecycle,
        *nav2,
        odometria_visual,
    ])
