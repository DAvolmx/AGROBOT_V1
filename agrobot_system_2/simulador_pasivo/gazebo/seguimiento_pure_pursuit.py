"""Image pixels -> camera ray -> ground plane -> pure-pursuit curvature.

Camera coordinates use the optical convention (+x right, +y down, +z forward).
The camera-to-robot extrinsic rotation is composed from the simulated pitch.
Camera height, pose and pitch are estimates until calibrated on the real robot.
"""
import math
from typing import Iterable, Optional, Tuple


def image_point_to_ground(center_px, row_px, width, height, *,
                          camera_height_m=0.49, camera_pitch_down_rad=0.25,
                          camera_forward_m=0.235, camera_left_m=0.0,
                          horizontal_fov_rad=1.22,
                          min_down_angle_rad=0.12, max_range_m=2.5):
    """Project an image point to (forward, left) meters in chassis coordinates."""
    if width <= 0 or height <= 0 or camera_height_m <= 0:
        return None
    # Intrinsic matrix K and K^-1 [u,v,1]^T (zero skew; square pixels).
    fx = width / (2.0 * math.tan(horizontal_fov_rad / 2.0))
    fy = fx
    cx, cy = (width - 1) / 2.0, (height - 1) / 2.0
    ray_cam_x = (center_px - cx) / fx
    ray_cam_y = (row_px - cy) / fy
    ray_cam_z = 1.0

    # R_robot_camera: optical right -> robot right, optical down is pitched
    # relative to robot z, and optical forward follows robot +x.
    p = camera_pitch_down_rad
    dx = math.cos(p) * ray_cam_z - math.sin(p) * ray_cam_y
    dy = -ray_cam_x
    dz = -math.sin(p) * ray_cam_z - math.cos(p) * ray_cam_y
    if dz >= -math.sin(min_down_angle_rad):
        return None
    ray_scale = camera_height_m / -dz
    if not math.isfinite(ray_scale) or ray_scale <= 0:
        return None
    forward = camera_forward_m + ray_scale * dx
    left = camera_left_m + ray_scale * dy
    if forward <= camera_forward_m or math.hypot(forward-camera_forward_m, left-camera_left_m) > max_range_m:
        return None
    return forward, left


def select_lookahead(centers: Iterable[Tuple[float, float]], width, height, *,
                     lookahead_m=0.70, **camera):
    """Select the detected lane center nearest the requested ground lookahead."""
    candidates = []
    for center_px, row_px in centers:
        point = image_point_to_ground(center_px, row_px, width, height, **camera)
        if point is None:
            continue
        forward, left = point
        distance = math.hypot(forward, left)
        if 0.30 <= forward <= 1.8:
            candidates.append((abs(distance - lookahead_m), forward, left))
    if not candidates:
        return None
    _, forward, left = min(candidates)
    return forward, left


def project_lane_points(centers, width, height, **camera):
    """Project all usable detected lane centers onto the local ground plane."""
    out = []
    for u, v in centers:
        point = image_point_to_ground(u, v, width, height, **camera)
        if point is not None and 0.25 <= point[0] <= 1.8:
            out.append(point)
    return out


def pure_pursuit_curvature(forward_m, left_m, *, wheelbase_m=0.37,
                           steering_limit_rad=math.radians(20.0)):
    """Return feasible path curvature (1/m) and equivalent steering angle."""
    if forward_m <= 0 or wheelbase_m <= 0:
        raise ValueError("lookahead forward distance and wheelbase must be positive")
    curvature = 2.0 * left_m / (forward_m * forward_m + left_m * left_m)
    max_curvature = math.tan(steering_limit_rad) / wheelbase_m
    curvature = max(-max_curvature, min(max_curvature, curvature))
    steering = math.atan(wheelbase_m * curvature)
    return curvature, steering


def estimate_path_tangent(points, target):
    """Estimate local path heading from projected ground points (robot frame)."""
    forward, _ = target
    valid = [(float(x), float(y)) for x, y in points
             if math.isfinite(x) and math.isfinite(y) and 0.25 <= x <= 1.8]
    if len(valid) < 2:
        return 0.0
    # Fit a local line around the chosen lookahead point; avoid distant image
    # rows that have much larger projection uncertainty.
    local = [(x, y) for x, y in valid if abs(x-forward) <= 0.45]
    if len(local) < 2:
        local = sorted(valid, key=lambda p: abs(p[0]-forward))[:3]
    xs, ys = zip(*local)
    mean_x = sum(xs) / len(xs)
    mean_y = sum(ys) / len(ys)
    denom = sum((x-mean_x)**2 for x in xs)
    if denom < 1e-6:
        return 0.0
    slope = sum((x-mean_x)*(y-mean_y) for x, y in local) / denom
    return math.atan(slope)


def offset_point_twist_command(pose, body_twist, target_local, path_heading_local,
                               *, offset_m=0.185, speed_ref=0.04,
                               kp=0.8, kd=0.0, mode="pv",
                               max_speed=0.04, max_curvature=math.tan(math.radians(20))/0.37):
    """PV/PD control for a point e ahead, then invert its Jacobian to (v,w).

    pose=(x,y,yaw), body_twist=(v,w), target_local=(x,y), all SI units.
    Returns (v,w,diagnostics). Odom is treated as the estimated robot state.
    """
    x, y, yaw = map(float, pose)
    v_est, w_est = map(float, body_twist)
    tx, ty = map(float, target_local)
    if offset_m <= 0 or mode not in ("pv", "pd"):
        raise ValueError("offset must be positive and mode must be pv or pd")

    c, s = math.cos(yaw), math.sin(yaw)
    eta = (x + offset_m*c, y + offset_m*s)
    # Transform the vision target and path tangent into world coordinates.
    eta_ref = (x + c*tx - s*ty, y + s*tx + c*ty)
    theta_ref = yaw + path_heading_local
    eta_dot_ref = (speed_ref*math.cos(theta_ref), speed_ref*math.sin(theta_ref))
    eta_dot = (v_est*c - offset_m*w_est*s,
               v_est*s + offset_m*w_est*c)
    err = (eta_ref[0]-eta[0], eta_ref[1]-eta[1])
    ux = eta_dot_ref[0] + kp*err[0]
    uy = eta_dot_ref[1] + kp*err[1]
    if mode == "pd":
        ux += kd*(eta_dot_ref[0]-eta_dot[0])
        uy += kd*(eta_dot_ref[1]-eta_dot[1])

    # Rotate desired point velocity to the chassis frame and invert J_e.
    vx_local = c*ux + s*uy
    vy_local = -s*ux + c*uy
    v_cmd = max(-max_speed, min(max_speed, vx_local))
    w_cmd = vy_local / offset_m
    w_limit = max_curvature * max(abs(v_cmd), 0.005)
    w_cmd = max(-w_limit, min(w_limit, w_cmd))
    return v_cmd, w_cmd, {
        "point_error_m": math.hypot(*err),
        "point_error_x_m": err[0], "point_error_y_m": err[1],
        "path_heading_local_rad": path_heading_local,
        "offset_m": offset_m,
    }
