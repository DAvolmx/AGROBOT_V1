"""
Puente ROS 2 <-> AgroBot backend.
==================================
Decisiones tomadas con el usuario antes de escribir esto:

  0. El robot es DIFERENCIAL (skid-steer): /cmd_vel solo aporta linear.x y
     angular.z. `linear.y` se IGNORA a propósito — el robot no puede
     desplazarse de costado, y un planner holonómico que lo pidiera haría
     patinar las ruedas sin desplazar nada.
  1. El enlace Jetson <-> ESP32-C3 SIGUE siendo el UDP binario propio
     (protocol.py) — no se migra a ros2_control/serial. Este puente solo
     SUSCRIBE /cmd_vel (lo que publica Nav2) y llama a la misma función
     `send_command()` que ya usaba el control manual, así que la evasión
     reactiva de slam.py se sigue aplicando como capa extra de seguridad
     incluso con Nav2 corriendo su propio costmap.
  2. El LIDAR real se lee con el driver oficial `sllidar_ros2` (nodo ROS
     aparte, ver ros2_ws/), no con el POST /api/lidar/scan de antes (ese
     endpoint se deja como fallback manual/pruebas).
  3. Se migra a Nav2 completo: mapeo con slam_toolbox, guardar el mapa, y
     navegar a metas reales (no solo evasión reactiva sin destino).
  4. Todo corre en la Jetson — este módulo vive embebido en el mismo
     proceso que jetson_server.py (un nodo rclpy en un hilo de fondo), sin
     workspace ROS separado para esta pieza. Los nodos "pesados"
     (sllidar_ros2, slam_toolbox, robot_localization, nav2) sí son
     procesos ROS aparte, lanzados por ros2_ws/src/agrobot_navigation/.

Si rclpy no está instalado (por ejemplo desarrollando en Windows sin ROS),
todo este módulo se vuelve un no-op silencioso y el sistema sigue
funcionando con el PoseEstimator/LidarReactive caseros de slam.py como
antes — así el mismo jetson_server.py corre con o sin ROS instalado.
"""
import math
import threading
import time

# Limites del envio de /map al dashboard. Ver _map_cb para el porque.
#   · 1 s entre envios: slam_toolbox publica cada 2 s, pero el mapa cambia
#     despacio y el ojo no distingue mas.
#   · 250.000 celdas ~ 500x500. Como lista de enteros de Python son unos 9 MB,
#     que es asumible; sin tope, un mapa de 2000x2000 son 145 MB por copia.
MAPA_PERIODO_S = 1.0
MAPA_MAX_CELDAS = 250_000


# El MOTIVO del fallo se guarda, no se descarta. Antes este `except` se
# tragaba la excepción y el dashboard mostraba "ROS no disponible" sin más:
# el caso habitual —lanzar el backend sin `source /opt/ros/jazzy/setup.bash`,
# con ROS perfectamente instalado— era indistinguible de no tener ROS. El
# usuario no tenía forma de saber cuál de los dos le estaba pasando.
ROS_UNAVAILABLE_REASON = ""
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist, PoseStamped
    from sensor_msgs.msg import Imu, LaserScan
    from nav_msgs.msg import Odometry, OccupancyGrid
    from nav2_msgs.action import NavigateToPose
    from tf2_ros import Buffer, TransformListener
    from action_msgs.msg import GoalStatus
    ROS_AVAILABLE = True
except ImportError as exc:
    ROS_AVAILABLE = False
    import os as _os
    _falta = exc.name or str(exc)
    if _os.environ.get("ROS_DISTRO"):
        ROS_UNAVAILABLE_REASON = (
            f"ROS_DISTRO={_os.environ['ROS_DISTRO']} está en el entorno pero "
            f"falta el módulo '{_falta}'. ¿Se instaló el paquete? "
            f"(ros-{_os.environ['ROS_DISTRO']}-nav2-msgs, -sensor-msgs, ...)")
    elif _os.path.isdir("/opt/ros"):
        _distros = sorted(d for d in _os.listdir("/opt/ros")
                          if _os.path.isfile(f"/opt/ros/{d}/setup.bash"))
        _cual = _distros[0] if _distros else "<distro>"
        ROS_UNAVAILABLE_REASON = (
            f"ROS está instalado (/opt/ros/{_cual}) pero el backend se lanzó "
            f"sin activarlo: falta '{_falta}'. Arrancalo con "
            f"`source /opt/ros/{_cual}/setup.bash` antes, o usá arrancar.sh")
    else:
        ROS_UNAVAILABLE_REASON = (
            f"ROS no está instalado en esta máquina (falta '{_falta}'). "
            f"El robot funciona igual con el SLAM casero de slam.py.")


class AgroBotROSBridge:
    """
    Wrapper sin rclpy en la superficie pública — jetson_server.py llama a estos
    métodos sin preocuparse de si ROS está disponible o no.
    """

    def __init__(self, on_cmd_vel, on_scan_points, on_fused_pose, on_map):
        self.available = ROS_AVAILABLE
        self._on_cmd_vel = on_cmd_vel            # callback(v_ref, omega_ref)
        self._on_scan_points = on_scan_points    # callback([{angle_deg, distance_m}, ...])
        self._on_fused_pose = on_fused_pose      # callback({x, y, theta_deg})
        self._on_map = on_map                    # callback(map_dict)
        self._node = None
        self._executor = None
        self._thread = None
        self.nav_goal_status = "sin misión"

        if not self.available:
            return

        rclpy.init(args=None)
        self._node = _BridgeNode(on_cmd_vel, on_scan_points, on_fused_pose, on_map)
        self._executor = rclpy.executors.SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()
        print("[ROS] puente iniciado — nodo 'agrobot_bridge' escuchando /cmd_vel, /scan, /odometry/filtered, /map")

    def publish_imu(self, accel: dict, gyro: dict):
        if self.available:
            self._node.publish_imu(accel, gyro)

    def publish_wheel_odom(self, v_ref: float, omega_ref: float):
        """
        No hay encoders reales: esto publica la velocidad COMANDADA como
        una 'odometría' de covarianza alta en /wheel_odom, para que
        robot_localization tenga una segunda fuente (además del IMU) con
        la que fusionar en su EKF — exactamente lo mismo que hacía
        PoseEstimator a mano, ahora delegado a un EKF de verdad.

        No se publica `linear.y`: el robot es diferencial y no tiene
        velocidad lateral. `ekf.yaml` tampoco la fusiona — marcarla como
        fusionable haría que el EKF tomara el cero implícito del mensaje
        como una medición firme de "no hubo deriva lateral", cuando en un
        skid-steer el patinaje sí produce algo de deriva que el EKF debe
        poder absorber por otras vías.
        """
        if self.available:
            self._node.publish_wheel_odom(v_ref, omega_ref)

    def send_nav_goal(self, x: float, y: float, theta_deg: float = 0.0) -> bool:
        if not self.available:
            return False
        return self._node.send_nav_goal(x, y, theta_deg)

    def cancel_nav_goal(self):
        if self.available:
            self._node.cancel_nav_goal()

    def estado_meta(self) -> str:
        """Cómo va la última meta enviada a Nav2 (ver _BridgeNode.estado_meta)."""
        if not self.available:
            return "sin_meta"
        return self._node.estado_meta

    def pose_en_mapa(self) -> dict | None:
        """Pose del robot en el marco `map`, o None si SLAM no publica TF."""
        if not self.available:
            return None
        return self._node.pose_en_mapa()

    def shutdown(self):
        if not self.available:
            return
        try:
            self._executor.shutdown()
            self._node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if ROS_AVAILABLE:
    class _BridgeNode(Node):
        def __init__(self, on_cmd_vel, on_scan_points, on_fused_pose, on_map):
            super().__init__("agrobot_bridge")
            self._on_cmd_vel = on_cmd_vel
            self._on_scan_points = on_scan_points
            self._on_fused_pose = on_fused_pose
            self._on_map = on_map

            # ---- Nav2 -> actuador (vía el mismo UDP binario de siempre) ----
            self.create_subscription(Twist, "/cmd_vel", self._cmd_vel_cb, 10)

            # ---- sllidar_ros2 -> radar del dashboard + evasión reactiva ----
            self.create_subscription(LaserScan, "/scan", self._scan_cb, qos_profile_sensor_data)

            # ---- robot_localization (EKF) -> pose fusionada real ----
            self.create_subscription(Odometry, "/odometry/filtered", self._odom_cb, 10)

            # ---- slam_toolbox -> mini-mapa del dashboard ----
            self._ultimo_mapa = 0.0
            self.create_subscription(OccupancyGrid, "/map", self._map_cb, 10)

            # ---- publicadores hacia robot_localization ----
            self.imu_pub = self.create_publisher(Imu, "/imu/data_raw", 10)
            self.wheel_odom_pub = self.create_publisher(Odometry, "/wheel_odom", 10)

            # ---- cliente de acción Nav2 para "ir a este punto del mapa" ----
            self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
            self._goal_handle = None
            self._meta_id = 0
            # "sin_meta" | "enviada" | "en_curso" | "exito" | "abortada" | "cancelada" | "rechazada"
            self.estado_meta = "sin_meta"

            # ---- TF: pose en el marco `map` para la patrulla ----
            # /odometry/filtered está en `odom`, pero las metas de Nav2 van
            # en `map`. Grabar en uno y navegar en el otro se separa con cada
            # corrección de slam_toolbox.
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)

        def _cmd_vel_cb(self, msg: "Twist"):
            # `msg.linear.y` se descarta: el robot es diferencial. Nav2 está
            # configurado como diferencial (`vy_samples` fuera, límites de y
            # en cero), así que en la práctica nunca llega distinto de cero;
            # si llegara, ejecutarlo solo haría patinar las ruedas.
            self._on_cmd_vel(float(msg.linear.x), float(msg.angular.z))

        def _scan_cb(self, msg: "LaserScan"):
            points = []
            angle = msg.angle_min
            for r in msg.ranges:
                if msg.range_min <= r <= msg.range_max:
                    points.append({"angle_deg": math.degrees(angle), "distance_m": float(r)})
                angle += msg.angle_increment
            self._on_scan_points(points)

        def _odom_cb(self, msg: "Odometry"):
            q = msg.pose.pose.orientation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            theta = math.atan2(siny_cosp, cosy_cosp)
            self._on_fused_pose({
                "x": msg.pose.pose.position.x,
                "y": msg.pose.pose.position.y,
                "theta_deg": math.degrees(theta) % 360,
            })

        def _map_cb(self, msg: "OccupancyGrid"):
            """
            Convierte /map a algo que el dashboard pueda dibujar, SIN reventar
            la memoria por el camino.

            Que arreglamos aqui, medido el 2026-09-06: la version anterior
            hacia `list(msg.data)` en CADA publicacion del mapa. slam_toolbox
            publica cada 2 s (`map_update_interval: 2.0`) y el mapa crece
            segun se explora. Una celda ocupa 1 byte en el mensaje de ROS y
            unos 36 como entero de Python dentro de una lista: un mapa de
            2000x2000 pasa de 4 MB a unos 145 MB, y eso cada dos segundos,
            mas la copia que hace json.dumps al mandarlo. Se vio al backend
            saltar de 1501 MB a 3535 MB y morir.

            Dos defensas:
              1. El limite de frecuencia se comprueba ANTES de convertir
                 nada. Antes se filtraba aguas abajo, en `_ros_on_map`, o
                 sea DESPUES de haber pagado la conversion entera.
              2. Un mapa grande se submuestrea en rejilla antes de enviarlo.
                 El dashboard lo dibuja en unos cientos de pixeles: mandarle
                 4 millones de celdas no le sirve de nada.
            """
            ahora = time.time()
            if ahora - self._ultimo_mapa < MAPA_PERIODO_S:
                return
            self._ultimo_mapa = ahora

            w, h, datos = msg.info.width, msg.info.height, msg.data
            res = msg.info.resolution

            # Paso de submuestreo: el menor que deje el mapa bajo el tope.
            paso = 1
            while w and h and (w // paso) * (h // paso) > MAPA_MAX_CELDAS:
                paso += 1

            if paso > 1:
                filas = range(0, h, paso)
                cols = range(0, w, paso)
                datos = [datos[y * w + x] for y in filas for x in cols]
                w, h = len(cols), len(filas)
                res = res * paso
            else:
                datos = list(datos)

            self._on_map({
                "width": w,
                "height": h,
                "resolution": res,
                "origin_x": msg.info.origin.position.x,
                "origin_y": msg.info.origin.position.y,
                "data": datos,
                "received_at": ahora,
            })

        def publish_imu(self, accel: dict, gyro: dict):
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "imu_link"
            msg.linear_acceleration.x = float(accel.get("x", 0.0)) * 9.80665
            msg.linear_acceleration.y = float(accel.get("y", 0.0)) * 9.80665
            msg.linear_acceleration.z = float(accel.get("z", 0.0)) * 9.80665
            msg.angular_velocity.x = math.radians(float(gyro.get("x", 0.0)))
            msg.angular_velocity.y = math.radians(float(gyro.get("y", 0.0)))
            msg.angular_velocity.z = math.radians(float(gyro.get("z", 0.0)))
            # orientación desconocida (no hay magnetómetro/fusión propia): covarianza -1 = "no la uses"
            msg.orientation_covariance[0] = -1.0
            self.imu_pub.publish(msg)

        def publish_wheel_odom(self, v_ref: float, omega_ref: float):
            msg = Odometry()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "odom"
            msg.child_frame_id = "base_link"
            msg.twist.twist.linear.x = float(v_ref)
            msg.twist.twist.angular.z = float(omega_ref)
            # covarianza alta a propósito: esto es velocidad COMANDADA, no medida
            # (sin encoders) — robot_localization debe confiar más en el IMU.
            # Índices 0/7/35 = vx/vy/vyaw en la matriz 6x6 aplanada.
            # vy se marca con covarianza alta igual, aunque no se publique:
            # deja constancia de que ese eje NO está medido.
            for i in (0, 7, 35):
                msg.twist.covariance[i] = 0.5
            self.wheel_odom_pub.publish(msg)

        def send_nav_goal(self, x: float, y: float, theta_deg: float) -> bool:
            if not self._nav_client.wait_for_server(timeout_sec=1.5):
                self.get_logger().warn("Nav2 action server no disponible")
                return False
            goal = NavigateToPose.Goal()
            goal.pose = PoseStamped()
            goal.pose.header.frame_id = "map"
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x = float(x)
            goal.pose.pose.position.y = float(y)
            theta = math.radians(theta_deg)
            goal.pose.pose.orientation.z = math.sin(theta / 2)
            goal.pose.pose.orientation.w = math.cos(theta / 2)
            # El handle hay que guardarlo, si no `cancel_nav_goal` no tiene
            # a quién cancelar y es un no-op silencioso: la meta seguía viva
            # y Nav2 seguía publicando /cmd_vel después de "cancelar".
            # Cada meta lleva un número: los callbacks de una meta vieja (ya
            # reemplazada) no deben pisar el estado de la actual.
            self._meta_id += 1
            meta_id = self._meta_id
            self.estado_meta = "enviada"
            futuro = self._nav_client.send_goal_async(goal)
            futuro.add_done_callback(lambda f: self._guardar_goal_handle(f, meta_id))
            return True

        def _guardar_goal_handle(self, futuro, meta_id):
            if meta_id != self._meta_id:
                return
            try:
                handle = futuro.result()
            except Exception as exc:
                self.get_logger().warn(f"Nav2 rechazó la meta: {exc}")
                self._goal_handle = None
                self.estado_meta = "rechazada"
                return
            if not handle.accepted:
                self._goal_handle = None
                self.estado_meta = "rechazada"
                return
            self._goal_handle = handle
            self.estado_meta = "en_curso"
            handle.get_result_async().add_done_callback(
                lambda f: self._guardar_resultado(f, meta_id))

        def _guardar_resultado(self, futuro, meta_id):
            if meta_id != self._meta_id:
                return
            try:
                status = futuro.result().status
            except Exception:
                status = GoalStatus.STATUS_ABORTED
            self.estado_meta = {
                GoalStatus.STATUS_SUCCEEDED: "exito",
                GoalStatus.STATUS_CANCELED: "cancelada",
            }.get(status, "abortada")

        def pose_en_mapa(self) -> dict | None:
            try:
                t = self._tf_buffer.lookup_transform(
                    "map", "base_link", rclpy.time.Time())
            except Exception:
                return None
            q = t.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                             1 - 2 * (q.y * q.y + q.z * q.z))
            return {
                "x": t.transform.translation.x,
                "y": t.transform.translation.y,
                "theta_deg": math.degrees(yaw) % 360,
            }

        def cancel_nav_goal(self):
            if self._goal_handle is not None:
                self._goal_handle.cancel_goal_async()
                self._goal_handle = None
