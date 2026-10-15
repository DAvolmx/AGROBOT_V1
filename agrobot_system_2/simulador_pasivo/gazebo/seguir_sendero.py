#!/usr/bin/env python3
"""Camera-first lane following smoke test for the Gazebo AgroBot.

Green crop rows or the raised-earth furrows define the visible lane; LiDAR
provides an emergency obstacle stop and wheel contacts trigger recovery.
Ground-truth odometry is used only for evaluation metrics, never for control.
"""
import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
from percepcion_sendero import detectar_centros
from seguimiento_pure_pursuit import (
    select_lookahead, pure_pursuit_curvature, project_lane_points,
    estimate_path_tangent, offset_point_twist_command,
)
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import Image, LaserScan

SIDES = ("front_left", "front_right", "rear_left", "rear_right")
# Camera is ~1.95 Hz wall-clock on the Jetson at RTF 0.65. Allow one dropped
# segmentation frame; speed decays while the last image estimate is held.
VISION_TIMEOUT_S = 1.05


class LanePilot(Node):
    def __init__(self, duration: float, gain: float, speed: float, output: Path,
                 lookahead: float = 0.70, controller: str = "pp",
                 kp: float = 0.8, kd: float = 0.35, offset: float = 0.185,
                 capture_dir: Path = None, require_ground_truth: bool = False):
        super().__init__("agrobot_lane_training")
        self.duration, self.gain, self.speed, self.output = duration, gain, speed, output
        self.lookahead = lookahead
        self.controller, self.kp, self.kd, self.offset = controller, kp, kd, offset
        self.capture_dir = capture_dir
        self.capture_times = (3.0, 12.0, 22.0)
        self.captures = []
        self.require_ground_truth = require_ground_truth
        self.truth_samples = []
        self.started = time.monotonic()
        self.cmd = self.create_publisher(Twist, "/agrobot/cmd_vel", 10)
        self.create_subscription(Image, "/agrobot/camera/image_raw", self.on_image, 1)
        self.create_subscription(LaserScan, "/agrobot/scan", self.on_scan, 1)
        self.create_subscription(Odometry, "/agrobot/odom", self.on_odom, 10)
        self.create_subscription(Odometry, "/agrobot/ground_truth/odom", self.on_truth, 10)
        for side in SIDES:
            self.create_subscription(Contacts, f"/agrobot/contact/{side}",
                                     lambda msg, s=side: self.on_contact(s, msg), 10)
        self.timer = self.create_timer(0.1, self.control)
        self.image = None
        self.camera_frames = 0
        self.valid_lane_frames = 0
        self.scan = None
        self.pose = None
        self.yaw = 0.0
        self.last_path_error = None
        self.path_error = None
        self.last_cmd_v = 0.0
        self.last_cmd_w = 0.0
        self.path_seen_at = 0.0
        self.rows_seen = 0
        self.rows_sum = 0
        self.camera_seen_updates = 0
        self.samples = []
        self.contact_latched = False
        self.contact_wheels = set()
        self.contact_events = 0
        self.recovery_until = 0.0
        self.recovery_turn = 0.0
        self.contact_log = []
        self.min_front = float("inf")
        self.command_count = 0
        self.translation_updates = 0
        self.green_lane_frames = 0
        self.furrow_lane_frames = 0
        self.lookahead_point = None
        self.path_curvature = None
        self.virtual_steering_rad = None
        self.ground_lane_points = []
        self.path_heading_local = 0.0
        self.odom_twist = (0.0, 0.0)
        self.offset_diag = None

    def on_image(self, msg):
        if msg.encoding.lower() not in ("rgb8", "bgr8"):
            return
        image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        image = image[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        if msg.encoding.lower() == "bgr8":
            image = image[:, :, ::-1]
        self.image = image.copy()
        self.camera_frames += 1
        elapsed = time.monotonic() - self.started
        if self.capture_dir is not None and self.capture_times:
            target = self.capture_times[0]
            if elapsed >= target:
                try:
                    from PIL import Image as PILImage
                    self.capture_dir.mkdir(parents=True, exist_ok=True)
                    path = self.capture_dir / f"{self.controller}_{int(target):02d}s.png"
                    PILImage.fromarray(image).save(path)
                    self.captures.append(str(path))
                    self.capture_times = self.capture_times[1:]
                except Exception as exc:
                    self.get_logger().warning(f"No se pudo guardar captura: {exc}")
                    self.capture_times = self.capture_times[1:]
        h, w, _ = image.shape
        centers, source = detectar_centros(image)
        if centers:
            self.valid_lane_frames += 1
            self.green_lane_frames += int("vegetacion" in source)
            self.furrow_lane_frames += int("surcos" in source)
            centers = centers[-6:]
            # Weight near-field rows more strongly; they are less sensitive to
            # perspective and directly indicate whether the wheels fit.
            weights = np.array([max(1.0, row / h) for _, row in centers])
            center_px = float(np.average([x for x, _ in centers], weights=weights))
            focal = w / (2.0 * math.tan(1.22 / 2.0))
            self.path_error = math.atan((center_px - w / 2.0) / focal)
            self.lookahead_point = select_lookahead(centers, w, h, lookahead_m=self.lookahead)
            self.ground_lane_points = project_lane_points(centers, w, h)
            if self.lookahead_point is not None:
                self.path_heading_local = estimate_path_tangent(
                    self.ground_lane_points, self.lookahead_point)
            self.path_seen_at = time.monotonic()
            self.rows_seen = len(centers)

    def on_scan(self, msg):
        self.scan = msg
        values = []
        for i, distance in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if abs(angle) <= math.radians(24) and math.isfinite(distance):
                if msg.range_min + 0.025 < distance < 0.8:
                    values.append(float(distance))
        self.min_front = min(values, default=float("inf"))

    def on_odom(self, msg):
        pose = msg.pose.pose
        self.pose = (pose.position.x, pose.position.y)
        q = pose.orientation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                             1 - 2 * (q.y * q.y + q.z * q.z))
        self.odom_twist = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)
        # The generated test world has a straight centerline at y=0. This is
        # a score only; no odometry enters the control law above.
        self.samples.append({"t": time.monotonic() - self.started,
                             "x": self.pose[0], "y": self.pose[1],
                             "yaw": self.yaw,
                             "cross_track_m": abs(self.pose[1]),
                             "vision_error_rad": self.path_error,
                             "lookahead_ground_m": self.lookahead_point,
                             "path_curvature_1pm": self.path_curvature,
                             "equivalent_steering_deg": None if self.virtual_steering_rad is None else round(math.degrees(self.virtual_steering_rad), 2),
                             "controller": self.controller,
                             "cmd_v_mps": self.last_cmd_v,
                             "cmd_w_rps": self.last_cmd_w,
                             "contact": self.contact_latched})

    def on_truth(self, msg):
        p = msg.pose.pose.position
        self.truth_samples.append({"t": time.monotonic() - self.started,
                                   "x": p.x, "y": p.y, "z": p.z})

    def on_contact(self, side, msg):
        contacts = getattr(msg, "contacts", [])
        obstacle_hit = any("suelo::" not in (c.collision1.name + c.collision2.name)
                           for c in contacts)
        was_latched = bool(self.contact_wheels)
        if obstacle_hit:
            self.contact_wheels.add(side)
        else:
            self.contact_wheels.discard(side)
        self.contact_latched = bool(self.contact_wheels)
        if obstacle_hit and not was_latched:
            self.contact_events += 1
            self.contact_latched = True
            self.contact_log.append({
                "t_s": round(time.monotonic() - self.started, 2),
                "wheel": side,
                "x_odom_m": None if self.pose is None else round(self.pose[0], 3),
                "y_odom_m": None if self.pose is None else round(self.pose[1], 3),
            })
            self.recovery_until = time.monotonic() + 1.15
            # Low objects are below the LiDAR plane. Choose the recovery turn
            # from the RGB lane error so the robot returns toward the corridor;
            # wheel-side naming alone can steer it farther out of the row.
            if self.path_error is not None and abs(self.path_error) > 0.015:
                # Pixels to the right have positive image error, whereas ROS
                # positive yaw turns left. Recovery must use the opposite sign.
                self.recovery_turn = -math.copysign(0.30, self.path_error)
            else:
                self.recovery_turn = self.clearer_turn()

    def clearer_turn(self):
        if self.scan is None:
            return 0.45
        sectors = {"left": [], "right": []}
        for i, distance in enumerate(self.scan.ranges):
            angle = self.scan.angle_min + i * self.scan.angle_increment
            if math.radians(22) < abs(angle) < math.radians(75) and math.isfinite(distance):
                if self.scan.range_min < distance < self.scan.range_max:
                    sectors["left" if angle > 0 else "right"].append(distance)
        left = float(np.median(sectors["left"])) if sectors["left"] else 0.0
        right = float(np.median(sectors["right"])) if sectors["right"] else 0.0
        return 0.45 if left >= right else -0.45

    def control(self):
        now = time.monotonic()
        elapsed = now - self.started
        msg = Twist()
        vision_age = now - self.path_seen_at
        # Keep only a short hold of the last valid heading. A dropped/blank
        # camera frame must not let the robot continue blindly for a full second.
        fresh_vision = self.path_error is not None and vision_age < VISION_TIMEOUT_S
        if fresh_vision:
            self.camera_seen_updates += 1
            self.rows_sum += self.rows_seen
        if now < self.recovery_until:
            phase = self.recovery_until - now
            msg.linear.x = -0.10 if phase > 0.65 else 0.0
            msg.angular.z = 0.0 if phase > 0.65 else self.recovery_turn
        elif self.min_front < 0.32:
            # Low vegetation remains camera territory; LiDAR only vetoes an
            # object actually intersecting its scan plane.
            msg.angular.z = self.clearer_turn()
        elif (fresh_vision and self.lookahead_point is not None
              and (self.controller == "pp" or self.pose is not None)):
            error = self.path_error
            age_scale = 1.0 if vision_age < 0.55 else 0.5 if vision_age < 0.85 else 0.25
            msg.linear.x = self.speed * age_scale * max(0.48, 1.0 - 0.7 * abs(error))
            forward, left = self.lookahead_point
            if self.controller == "pp":
                curvature, steering = pure_pursuit_curvature(forward, left)
                msg.angular.z = float(np.clip(msg.linear.x * curvature, -0.55, 0.55))
            else:
                msg.linear.x, msg.angular.z, diag = offset_point_twist_command(
                    (self.pose[0], self.pose[1], self.yaw), self.odom_twist,
                    self.lookahead_point, self.path_heading_local,
                    offset_m=self.offset, speed_ref=msg.linear.x, kp=self.kp,
                    kd=self.kd if self.controller == "pd" else 0.0,
                    mode=self.controller, max_speed=self.speed)
                self.offset_diag = diag
                curvature = msg.angular.z / max(abs(msg.linear.x), 1e-5)
                steering = math.atan(0.37 * curvature)
            self.path_curvature, self.virtual_steering_rad = curvature, steering
            self.last_path_error = error
        else:
            # Fail closed when crop rows cannot be segmented; do not continue
            # blindly on the orchard ground.
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        self.cmd.publish(msg)
        self.last_cmd_v, self.last_cmd_w = msg.linear.x, msg.angular.z
        self.command_count += 1
        self.translation_updates += int(msg.linear.x > 0.005)
        if elapsed >= self.duration:
            self.finish()

    def finish(self):
        self.cmd.publish(Twist())
        scoring_samples = self.truth_samples or ([] if self.require_ground_truth else self.samples)
        ys = np.array([row["y"] for row in scoring_samples], dtype=float)
        xs = np.array([row["x"] for row in scoring_samples], dtype=float)
        cross = np.abs(ys)
        result = {
            "duration_s": round(time.monotonic() - self.started, 2),
            "controller": f"RGB projected-ground {self.controller.upper()} + LiDAR stop + contact recovery",
            "lookahead_m": self.lookahead, "speed_mps": self.speed,
            "kp_s_inv": self.kp if self.controller in ("pv", "pd") else None,
            "kd": self.kd if self.controller == "pd" else None,
            "offset_point_m": self.offset if self.controller in ("pv", "pd") else None,
            "camera_rows_valid_mean": round(self.rows_sum / max(self.camera_seen_updates, 1), 2),
            "camera_seen_pct": round(100 * self.camera_seen_updates / max(self.command_count, 1), 1),
            "camera_frames": self.camera_frames,
            "camera_lane_frame_pct": round(100 * self.valid_lane_frames / max(self.camera_frames, 1), 1),
            "camera_blank_frames": max(0, self.camera_frames - self.valid_lane_frames),
            "camera_captures": self.captures,
            "camera_frames_with_lane": self.valid_lane_frames,
            "camera_green_lane_frames": self.green_lane_frames,
            "camera_furrow_lane_frames": self.furrow_lane_frames,
            "cross_track_rmse_cm": round(float(np.sqrt(np.mean(cross ** 2)) * 100), 2) if len(cross) else None,
            "cross_track_p95_cm": round(float(np.percentile(cross, 95) * 100), 2) if len(cross) else None,
            "cross_track_max_cm": round(float(np.max(cross) * 100), 2) if len(cross) else None,
            "within_6cm_pct": round(float(np.mean(cross <= 0.06) * 100), 1) if len(cross) else None,
            "distance_x_m": round(float(xs[-1] - xs[0]), 3) if len(xs) > 1 else 0.0,
            "progress_m_per_min": round(float((xs[-1] - xs[0]) / max(time.monotonic() - self.started, 1e-6) * 60), 3) if len(xs) > 1 else 0.0,
            "translation_command_duty_pct": round(100 * self.translation_updates / max(self.command_count, 1), 1),
            "obstacle_contact_events": self.contact_events,
            "contact_log": self.contact_log,
            "max_virtual_steering_deg": 20.0,
            "camera_ground_projection": "pinhole_K_inverse_ray_plane_intersection",
            "camera_extrinsics_status": "estimated_simulator_values_not_hardware_calibration",
            "min_lidar_front_m": None if not math.isfinite(self.min_front) else round(self.min_front, 3),
            "odom_samples": len(self.samples), "control_updates": self.command_count,
            "trajectory": self.samples,
            "ground_truth_trajectory": self.truth_samples,
            "evaluation_pose_source": "gazebo_ground_truth" if self.truth_samples else "wheel_odometry_not_ground_truth",
            "ground_truth_required": self.require_ground_truth,
            "ground_truth_used_for_control": False,
            "real_robot_deployment": False,
            "success": bool(
                len(xs) > 1
                and (xs[-1] - xs[0]) >= 0.5
                and self.valid_lane_frames / max(self.camera_frames, 1) >= 0.90
                and float(np.mean(cross <= 0.06)) >= 0.90
                and self.contact_events == 0
            ),
        }
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.output.write_text(json.dumps(result, indent=2), encoding="utf-8")
        summary = {k: v for k, v in result.items() if k != "trajectory"}
        self.get_logger().info(json.dumps(summary))
        self.done = True


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--duration", type=float, default=15.0)
    p.add_argument("--gain", type=float, default=1.1,
                   help="compatibility with old launch commands; unused by pure pursuit")
    p.add_argument("--speed", type=float, default=0.04)
    p.add_argument("--lookahead", type=float, default=0.70,
                   help="ground-projected pure-pursuit distance in meters")
    p.add_argument("--controller", choices=("pp", "pv", "pd"), default="pp",
                   help="pp=pure pursuit; pv/pd=offset-point visual path tracking")
    p.add_argument("--kp", type=float, default=0.8, help="position gain (1/s) for PV/PD")
    p.add_argument("--kd", type=float, default=0.35, help="velocity damping gain for PD")
    p.add_argument("--offset", type=float, default=0.185,
                   help="control point distance ahead of chassis center (m)")
    p.add_argument("--salida", type=Path, default=Path("/tmp/agrobot_lane_metrics.json"))
    p.add_argument("--capturas", type=Path, default=None,
                   help="directorio opcional para guardar tres fotogramas RGB del entrenamiento")
    p.add_argument("--require-ground-truth", action="store_true",
                   help="solo puntuar con pose real Gazebo; no usar odometría de ruedas como verdad")
    a = p.parse_args()
    rclpy.init()
    node = LanePilot(a.duration, a.gain, a.speed, a.salida, a.lookahead,
                     a.controller, a.kp, a.kd, a.offset, a.capturas, a.require_ground_truth)
    node.done = False
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.cmd.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
