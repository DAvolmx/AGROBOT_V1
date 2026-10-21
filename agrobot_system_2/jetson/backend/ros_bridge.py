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
import base64
import math
import threading
import time

import numpy as np

# Limites del envio de /map al dashboard. Ver _map_cb para el porque.
#   · 1 s entre envios: slam_toolbox publica cada 2 s, pero el mapa cambia
#     despacio y el ojo no distingue mas.
#   · 250.000 celdas ~ 500x500. Como lista de enteros de Python son unos 9 MB,
#     que es asumible; sin tope, un mapa de 2000x2000 son 145 MB por copia.
MAPA_PERIODO_S = 0.5
MAPA_MAX_CELDAS = 250_000
# Un mapa de una casa o una huerta no pasa de unas decenas de metros. El
# 2026-09-30 salió uno de 9,7 km x 21 m (193868 x 426 celdas, 82 MB por
# mensaje): el EKF había seguido integrando la última velocidad durante las
# horas sin LIDAR. Por encima de esto el mapa se marca como corrupto.
MAPA_EXTENSION_SOSPECHOSA_M = 500.0

# Covarianza de la velocidad cero que se publica en /zupt con el robot
# quieto. Firme (sigma 0,1 m/s) pero no absoluta: si alguien empuja el robot
# a mano, rf2o todavía pesa.
ZUPT_COVARIANZA = 0.001
# Quieto es quieto: el giro en reposo es cero con mucha más certeza de la que
# da el giroscopio (varianza IMU_VAR_GIRO), así que en reposo manda el ZUPT.
ZUPT_COVARIANZA_GIRO = 1e-5
IMU_VAR_GIRO = math.radians(2.4) ** 2

# Giro del LIDAR respecto al robot. Con el C1 montado según la flecha de la
# carcasa, el 0° de sllidar_ros2 mira HACIA ATRÁS (2026-09-19: una persona a
# 90 cm detrás salía en 0°). Debe coincidir con el yaw de LIDAR_RPY en
# ros2_ws/src/agrobot_navigation/launch/*.launch.py.
LIDAR_YAW_DEG = 180.0


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
    from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
    from sensor_msgs.msg import Imu, LaserScan
    from nav_msgs.msg import Odometry, OccupancyGrid
    from nav2_msgs.action import NavigateToPose
    from tf2_ros import Buffer, TransformListener
    from action_msgs.msg import GoalStatus
    from ament_index_python.packages import get_package_share_directory as _share
    ROS_AVAILABLE = True

    # Árbol de comportamiento que Nav2 debe ejecutar para cada meta.
    #
    # Se manda en la META (campo `behavior_tree`) y no solo por parámetro: el
    # 2026-09-17 `bt_navigator` tenía `default_nav_to_pose_bt_xml: ""` y en
    # Jazzy eso NO significa "el de por defecto" sino árbol VACÍO. Aceptaba
    # cada meta y la abortaba al instante ("Behavior tree threw exception:
    # Empty Tree"): 63 metas seguidas sin que el robot se moviera nunca.
    import os as _os
    BT_POR_DEFECTO = _os.path.join(
        _share("nav2_bt_navigator"), "behavior_trees",
        "navigate_to_pose_w_replanning_and_recovery.xml")
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

    def __init__(self, on_cmd_vel, on_scan_points, on_fused_pose, on_map,
                 on_odom_lidar=None):
        self.available = ROS_AVAILABLE
        self._on_cmd_vel = on_cmd_vel            # callback(v_ref, omega_ref)
        self._on_scan_points = on_scan_points    # callback([{angle_deg, distance_m}, ...])
        self._on_fused_pose = on_fused_pose      # callback({x, y, theta_deg})
        self._on_map = on_map                    # callback(map_dict)
        self._node = None
        self._executor = None
        self._thread = None
        self._parando = False
        self.fallos_spin = 0
        self.nav_goal_status = "sin misión"

        if not self.available:
            return

        rclpy.init(args=None)
        self._node = _BridgeNode(on_cmd_vel, on_scan_points, on_fused_pose, on_map,
                                 on_odom_lidar)
        self._executor = rclpy.executors.SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._girar_vigilado, daemon=True)
        self._thread.start()
        print("[ROS] puente iniciado — nodo 'agrobot_bridge' escuchando /cmd_vel, /scan, /odometry/filtered, /map")

    def _girar_vigilado(self):
        """Mantiene vivo el hilo del ejecutor pase lo que pase.

        Una excepción dentro de un callback de ROS mataba este hilo y el
        backend se quedaba sin LIDAR, sin mapa y sin navegación, pero con el
        proceso vivo: desde fuera parecía que "el sistema se cae" sin motivo
        (2026-09-17, un RuntimeError en el callback de /cmd_vel). Ahora se
        registra y se sigue girando."""
        while not self._parando:
            try:
                self._executor.spin()
                return                      # salida limpia
            except Exception as exc:
                self.fallos_spin += 1
                print(f"[ROS] el ejecutor falló ({type(exc).__name__}: {exc}); "
                      f"reintento {self.fallos_spin}", flush=True)
                time.sleep(0.5)

    @property
    def vivo(self) -> bool:
        """El puente sigue atendiendo callbacks de ROS."""
        return bool(self.available and self._thread is not None and self._thread.is_alive())

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

    def publish_odom_webcam(self, vx: float, vyaw: float, var_vx: float, var_vyaw: float):
        """Odometría visual de la webcam -> /odom_webcam (odom3 del EKF).
        Se llama desde el hilo de la odometría; `publish` de rclpy lo admite."""
        if self.available:
            self._node.publish_odom_webcam(vx, vyaw, var_vx, var_vyaw)

    def publish_zupt(self):
        """Velocidad CERO medida -> /zupt (odom4 del EKF).

        Solo se llama con el robot quieto de verdad (ver `_bucle_zupt` en
        jetson_server.py). Sin esto, cuando el LIDAR callaba, el EKF no
        recibía ninguna velocidad lineal y extrapolaba la última durante
        horas: el 2026-09-30 la pose llegó a x = 1771 m y el mapa a 9,7 km.
        """
        if self.available:
            self._node.publish_zupt()

    def publish_initial_pose(self, x: float, y: float, theta_deg: float):
        """Publica /initialpose para corregir la pose del robot en SLAM."""
        if self.available:
            self._node.publish_initial_pose(x, y, theta_deg)

    def pose_en_mapa(self) -> dict | None:
        """Pose del robot en el marco `map`, o None si SLAM no publica TF."""
        if not self.available:
            return None
        return self._node.pose_en_mapa()

    def shutdown(self):
        if not self.available:
            return
        self._parando = True
        try:
            self._executor.shutdown()
            self._node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if ROS_AVAILABLE:
    class _BridgeNode(Node):
        def __init__(self, on_cmd_vel, on_scan_points, on_fused_pose, on_map,
                     on_odom_lidar=None):
            super().__init__("agrobot_bridge")
            self._on_odom_lidar = on_odom_lidar
            self._on_cmd_vel = on_cmd_vel
            self._on_scan_points = on_scan_points
            self._on_fused_pose = on_fused_pose
            self._on_map = on_map

            # ---- Nav2 -> actuador (vía el mismo UDP binario de siempre) ----
            self.create_subscription(
                Twist, "/cmd_vel", lambda m: self._seguro("cmd_vel", self._cmd_vel_cb, m), 10)

            # ---- sllidar_ros2 -> radar del dashboard + evasión reactiva ----
            self.create_subscription(
                LaserScan, "/scan", lambda m: self._seguro("scan", self._scan_cb, m),
                qos_profile_sensor_data)

            # ---- robot_localization (EKF) -> pose fusionada real ----
            self.create_subscription(
                Odometry, "/odometry/filtered", lambda m: self._seguro("odom", self._odom_cb, m), 10)

            # ---- rf2o -> velocidad MEDIDA por el LIDAR, sin mezclar ----
            # La de /odometry/filtered no sirve para detectar patinaje: el
            # EKF ya fusionó ahí la velocidad ORDENADA (/wheel_odom).
            if on_odom_lidar is not None:
                self.create_subscription(
                    Odometry, "/odom_rf2o",
                    lambda m: self._seguro("odom_rf2o", self._odom_lidar_cb, m), 10)

            # ---- slam_toolbox -> mini-mapa del dashboard ----
            self._ultimo_mapa = 0.0
            # Cola de UNO: con 10, un mapa corrupto de 82 MB dejaba hasta
            # 800 MB retenidos en la cola del suscriptor (2026-09-30).
            self.create_subscription(
                OccupancyGrid, "/map", lambda m: self._seguro("map", self._map_cb, m), 1)

            # ---- publicadores hacia robot_localization ----
            self.imu_pub = self.create_publisher(Imu, "/imu/data_raw", 10)
            self.wheel_odom_pub = self.create_publisher(Odometry, "/wheel_odom", 10)
            self.odom_webcam_pub = self.create_publisher(Odometry, "/odom_webcam", 10)
            self.zupt_pub = self.create_publisher(Odometry, "/zupt", 10)

            # ---- publicador /initialpose para corregir la pose en SLAM ----
            self.initial_pose_pub = self.create_publisher(
                PoseWithCovarianceStamped, "/initialpose", 10)

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
            self._fallos = {}

        def _seguro(self, nombre, fn, *args):
            """Un callback que revienta mata el hilo del ejecutor y con él
            todo ROS dentro del backend (pasó el 2026-09-17). Se registra y
            se sigue."""
            try:
                fn(*args)
            except Exception as exc:
                self._fallos[nombre] = self._fallos.get(nombre, 0) + 1
                if self._fallos[nombre] <= 3:
                    self.get_logger().error(
                        f"callback {nombre} falló ({type(exc).__name__}: {exc})")

        def _cmd_vel_cb(self, msg: "Twist"):
            # `msg.linear.y` se descarta: el robot es diferencial. Nav2 está
            # configurado como diferencial (`vy_samples` fuera, límites de y
            # en cero), así que en la práctica nunca llega distinto de cero;
            # si llegara, ejecutarlo solo haría patinar las ruedas.
            self._on_cmd_vel(float(msg.linear.x), float(msg.angular.z))

        def _scan_cb(self, msg: "LaserScan"):
            # El scan viene en el frame `laser`; el freno y el radar trabajan
            # en el del robot (0° = adelante). Ver LIDAR_YAW_DEG.
            points = []
            angle = msg.angle_min
            for r in msg.ranges:
                if msg.range_min <= r <= msg.range_max:
                    grados = (math.degrees(angle) + LIDAR_YAW_DEG + 180.0) % 360.0 - 180.0
                    points.append({"angle_deg": grados, "distance_m": float(r)})
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
                # velocidades del EKF: el dashboard las pone al lado de las de
                # la webcam para poder calibrarla antes de fusionarla
                "vx": msg.twist.twist.linear.x,
                "vyaw": msg.twist.twist.angular.z,
                # Varianza de la velocidad de avance: acotada mientras llegue
                # alguna medida (rf2o o /zupt). Si crece sin freno el EKF está
                # extrapolando la última velocidad: así se estiró el mapa a
                # 9,7 km el 2026-09-30. (La de posición NO sirve: crece
                # siempre, nadie mide la velocidad lateral.)
                "var_vx": msg.twist.covariance[0],
            })

        def _odom_lidar_cb(self, msg: "Odometry"):
            self._on_odom_lidar(float(msg.twist.twist.linear.x),
                                float(msg.twist.twist.angular.z))

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
            extension_m = max(w, h) * res

            # Paso de submuestreo: el menor que deje el mapa bajo el tope.
            paso = 1
            while w and h and (w // paso) * (h // paso) > MAPA_MAX_CELDAS:
                paso += 1

            # numpy: reordenar 250.000 celdas con listas de Python eran
            # ~30 ms por mapa; con vistas son microsegundos.
            rejilla = np.frombuffer(bytes(datos), dtype=np.int8).reshape(h, w) if w and h \
                else np.zeros((0, 0), dtype=np.int8)
            if paso > 1:
                rejilla = rejilla[::paso, ::paso]
                h, w = rejilla.shape
                res = res * paso
            datos = np.ascontiguousarray(rejilla).tobytes()   # -1 queda como 255

            self._on_map({
                "width": w,
                "height": h,
                "resolution": res,
                "origin_x": msg.info.origin.position.x,
                "origin_y": msg.info.origin.position.y,
                # Binario en base64 (1 byte/celda, unas 8x menos que la lista
                # JSON) y sin crear 250.000 enteros de Python por mensaje.
                "data_b64": base64.b64encode(datos).decode("ascii"),
                "received_at": ahora,
                "extension_m": extension_m,
                "corrupto": extension_m > MAPA_EXTENSION_SOSPECHOSA_M,
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
            # Ruido REAL del giroscopio (MPU6050 en la LoRa V3, medido en
            # reposo el 2026-09-30: sigma 2,4 °/s). Con covarianza 0 el EKF
            # lo tomaba por perfecto y ninguna otra medida (ni el ZUPT) podía
            # corregirlo: con el sesgo sin calibrar giraba 53°/min quieto.
            for i in (0, 4, 8):
                msg.angular_velocity_covariance[i] = IMU_VAR_GIRO
                msg.linear_acceleration_covariance[i] = 0.1
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
            # 5 s y no 1.5: descubrir el servidor tardó 2,25 s medidos en el
            # robot, así que el primer intento fallaba siempre.
            if not self._nav_client.wait_for_server(timeout_sec=5.0):
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
            goal.behavior_tree = BT_POR_DEFECTO
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

        def publish_zupt(self):
            msg = Odometry()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "odom"
            msg.child_frame_id = "base_link"
            # vx y vyaw en cero; el resto de la matriz con varianza alta
            for i in range(6):
                msg.twist.covariance[i * 7] = 1e3
            msg.twist.covariance[0] = ZUPT_COVARIANZA     # vx
            msg.twist.covariance[7] = ZUPT_COVARIANZA     # vy
            msg.twist.covariance[35] = ZUPT_COVARIANZA_GIRO   # vyaw
            self.zupt_pub.publish(msg)

        def publish_odom_webcam(self, vx, vyaw, var_vx, var_vyaw):
            msg = Odometry()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "odom"
            msg.child_frame_id = "base_link"
            msg.twist.twist.linear.x = float(vx)
            msg.twist.twist.angular.z = float(vyaw)
            # Solo se miden vx y el giro; el resto con varianza enorme para
            # que nadie lo tome como una medida de "cero".
            for i in range(36):
                msg.twist.covariance[i] = 0.0
            for i in (7, 14, 21, 28):
                msg.twist.covariance[i] = 1e6
            msg.twist.covariance[0] = float(var_vx)
            msg.twist.covariance[35] = float(var_vyaw)
            self.odom_webcam_pub.publish(msg)

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

        def publish_initial_pose(self, x: float, y: float, theta_deg: float):
            msg = PoseWithCovarianceStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.pose.pose.position.x = float(x)
            msg.pose.pose.position.y = float(y)
            theta = math.radians(theta_deg)
            msg.pose.pose.orientation.z = math.sin(theta / 2)
            msg.pose.pose.orientation.w = math.cos(theta / 2)
            msg.pose.covariance[0] = 0.25
            msg.pose.covariance[7] = 0.25
            msg.pose.covariance[35] = 0.068
            self.initial_pose_pub.publish(msg)

        def cancel_nav_goal(self):
            if self._goal_handle is not None:
                self._goal_handle.cancel_goal_async()
                self._goal_handle = None
