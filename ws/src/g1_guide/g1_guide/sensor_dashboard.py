"""OpenCV ROS 2 dashboard for the G1's live sensor topics."""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
from typing import Any, Optional

import cv2
import numpy as np

from .frame_sync import FrameSynchronizer
from .g1_camera_viewer import (
    DEPTH_DISPLAY_MAX_M,
    _message_stamp_seconds,
    depth_colormap,
    depth_to_meters,
)
from .sensor_metrics import (
    MetricResult,
    StreamHealth,
    StreamTracker,
    dashboard_joint_line,
    dashboard_requires_display,
    dashboard_vector_rows,
    summarize_imu,
    summarize_joint_state,
    summarize_laser_scan,
    summarize_odometry,
)


WINDOW_NAME = "G1 Sensor Dashboard"
STALE_LIMITS = {
    "camera_rgb": 0.75,
    "camera_depth": 0.75,
    "scan": 1.0,
    "imu": 1.0,
    "odom": 1.5,
    "joint_states": 1.5,
}
CORE_STREAMS = ("camera_rgb", "camera_depth", "scan", "imu", "odom")
STATUS_COLORS = {
    "fresh": (70, 210, 90),
    "stale": (0, 150, 255),
    "missing": (0, 0, 255),
    "invalid": (0, 0, 255),
}


def _header_stamp(message: Any) -> Optional[float]:
    try:
        return _message_stamp_seconds(message)
    except (AttributeError, TypeError, ValueError):
        return None


def _has_display() -> bool:
    if sys.platform.startswith("win") or sys.platform == "darwin":
        return True
    return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))


def _put_text(
    image: np.ndarray,
    text: str,
    origin: tuple[int, int],
    color: tuple[int, int, int] = (230, 230, 230),
    scale: float = 0.55,
) -> None:
    _put_column_text(image, text, origin, image.shape[1] - origin[0] - 8, color, scale)


def _put_column_text(
    image: np.ndarray,
    text: str,
    origin: tuple[int, int],
    width: int,
    color: tuple[int, int, int] = (230, 230, 230),
    scale: float = 0.55,
) -> None:
    shown = str(text).encode("ascii", "replace").decode("ascii")
    available = max(1, min(width, image.shape[1] - origin[0] - 2))
    while shown and cv2.getTextSize(shown, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)[0][0] > available:
        shown = shown[:-4] + "..." if len(shown) > 4 else shown[:-1]
    cv2.putText(image, shown, origin, cv2.FONT_HERSHEY_SIMPLEX,
                scale, color, 1, cv2.LINE_AA)


def _fit_image(image: Optional[np.ndarray], width: int, height: int) -> np.ndarray:
    panel = np.zeros((height, width, 3), dtype=np.uint8)
    if image is None or image.ndim != 3 or image.shape[2] != 3 or image.size == 0:
        _put_text(panel, "WAITING FOR VALID FRAME", (16, height // 2), (0, 170, 255))
        return panel
    scale = min(width / image.shape[1], height / image.shape[0])
    resized = cv2.resize(image, (max(1, int(image.shape[1] * scale)),
                                 max(1, int(image.shape[0] * scale))))
    y = (height - resized.shape[0]) // 2
    x = (width - resized.shape[1]) // 2
    panel[y:y + resized.shape[0], x:x + resized.shape[1]] = resized
    return panel


def _lidar_panel(points: tuple[tuple[float, float], ...], max_range: float,
                 width: int = 430, height: int = 430) -> np.ndarray:
    panel = np.zeros((height, width, 3), dtype=np.uint8)
    center = (width // 2, height // 2)
    radius = min(width, height) // 2 - 24
    scale = radius / max(max_range, 0.1)
    for fraction in (0.25, 0.5, 0.75, 1.0):
        ring = int(radius * fraction)
        cv2.circle(panel, center, ring, (65, 65, 65), 1, cv2.LINE_AA)
    cv2.line(panel, (center[0] - radius, center[1]),
             (center[0] + radius, center[1]), (60, 60, 60), 1)
    cv2.line(panel, (center[0], center[1] - radius),
             (center[0], center[1] + radius), (60, 60, 60), 1)
    cv2.circle(panel, center, 5, (230, 230, 230), -1)
    for x_m, y_m in points:
        x = int(center[0] + x_m * scale)
        y = int(center[1] - y_m * scale)
        if 0 <= x < width and 0 <= y < height:
            cv2.circle(panel, (x, y), 2, (80, 220, 100), -1)
    _put_text(panel, f"0 m       range {max_range:.1f} m", (12, height - 10), (180, 180, 180), 0.42)
    return panel


def render_dashboard(
    now_monotonic: float,
    health: dict[str, StreamHealth],
    rgb: Optional[np.ndarray],
    depth: Optional[np.ndarray],
    frame_status: str,
    metrics: dict[str, MetricResult],
) -> np.ndarray:
    """Build the complete BGR dashboard image used by UI and snapshots."""
    width, height = 1500, 1010
    canvas = np.zeros((height, width, 3), dtype=np.uint8)
    canvas[:] = (24, 27, 31)
    _put_text(canvas, "G1 LIVE SENSOR TELEMETRY", (20, 30), (245, 245, 245), 0.8)

    x0, y0, view_w, view_h = 20, 55, 490, 350
    _put_text(canvas, f"RGB  [{health['camera_rgb'].status.upper()}]", (x0, y0 - 8),
              STATUS_COLORS.get(health["camera_rgb"].status, (0, 170, 255)))
    _put_text(canvas, f"DEPTH  [{health['camera_depth'].status.upper()}]  {frame_status}",
              (x0 + view_w + 20, y0 - 8), STATUS_COLORS.get(health["camera_depth"].status, (0, 170, 255)))
    canvas[y0:y0 + view_h, x0:x0 + view_w] = _fit_image(rgb, view_w, view_h)
    depth_color = depth_colormap(depth, DEPTH_DISPLAY_MAX_M) if depth is not None else None
    canvas[y0:y0 + view_h, x0 + view_w + 20:x0 + 2 * view_w + 20] = _fit_image(depth_color, view_w, view_h)

    _put_column_text(canvas, "LIDAR OCCUPANCY / RANGE OVERVIEW", (1050, 48), 430,
                     (230, 230, 230), 0.52)
    scan_result = metrics.get("scan")
    if health["scan"].status == "fresh" and scan_result and scan_result.valid:
        scan_values = scan_result.values
        lidar = _lidar_panel(scan_values["points_xy"], scan_values["range_max_m"])
        near = scan_values["nearest_m"]
        hit_line = f"finite hits {scan_values['hit_count']}/{scan_values['sample_count']}"
        range_line = (f"usable incl free {scan_values['usable_count']}; nearest {near:.2f} m"
                      if near is not None else
                      f"usable incl free {scan_values['usable_count']}; no obstacles")
    else:
        lidar = _lidar_panel((), 5.0)
        hit_line = "No valid scan data"
        range_line = ""
    canvas[60:490, 1050:1480] = lidar
    _put_column_text(canvas, hit_line, (1050, 510), 430, (190, 190, 190), 0.48)
    _put_column_text(canvas, range_line, (1050, 530), 430, (190, 190, 190), 0.48)

    _put_text(canvas, "STREAM HEALTH (monotonic receipt age; header stamp shown for context)",
              (20, 445), (220, 220, 220), 0.56)
    health_y = 470
    labels = {
        "camera_rgb": "/camera/image", "camera_depth": "/camera/depth_image",
        "scan": "/scan", "imu": "/imu", "odom": "/odom",
        "joint_states": "/joint_states",
    }
    for key in labels:
        state = health[key]
        age = "-" if state.age_sec is None else f"{state.age_sec:.2f}s"
        stamp = "-" if state.header_stamp_sec is None else f"{state.header_stamp_sec:.3f}s"
        line = f"{labels[key]:22} {state.status.upper():8} age={age:>6} stamp={stamp:>12}  {state.detail}"
        _put_column_text(canvas, line, (25, health_y), 1000,
                         STATUS_COLORS.get(state.status, (180, 180, 180)), 0.48)
        health_y += 27

    _put_text(canvas, "IMU", (20, 665), (100, 210, 255), 0.65)
    imu = metrics.get("imu")
    if health["imu"].status == "fresh" and imu and imu.valid:
        v = imu.values
        orient = v["orientation_xyzw"]
        imu_rows = ["orientation xyzw:", "unavailable"] if orient is None else list(
            dashboard_vector_rows("orientation xyzw:", orient)
        )
        imu_rows.extend(dashboard_vector_rows("gyro xyz (rad/s):", v["angular_velocity_xyz"]))
        imu_rows.extend(dashboard_vector_rows("acceleration xyz (m/s^2):", v["linear_acceleration_xyz"]))
        for row, text in enumerate(imu_rows):
            _put_column_text(canvas, text, (20, 695 + row * 21), 470, scale=0.48)
    else:
        detail = imu.detail if imu else health["imu"].detail
        _put_column_text(canvas, f"{health['imu'].status.upper()}: {detail}",
                         (20, 700), 470, (0, 130, 255))

    _put_text(canvas, "ODOMETRY", (520, 665), (100, 210, 255), 0.65)
    odom = metrics.get("odom")
    if health["odom"].status == "fresh" and odom and odom.valid:
        v = odom.values
        odom_rows = []
        odom_rows.extend(dashboard_vector_rows("position xyz (m):", v["position_xyz"]))
        odom_rows.extend(dashboard_vector_rows("orientation xyzw:", v["orientation_xyzw"]))
        odom_rows.extend(dashboard_vector_rows("linear velocity xyz (m/s):", v["linear_velocity_xyz"]))
        odom_rows.extend(dashboard_vector_rows("angular velocity xyz (rad/s):", v["angular_velocity_xyz"]))
        for row, text in enumerate(odom_rows):
            _put_column_text(canvas, text, (520, 695 + row * 21), 500, scale=0.48)
    else:
        detail = odom.detail if odom else health["odom"].detail
        _put_column_text(canvas, f"{health['odom'].status.upper()}: {detail}",
                         (520, 700), 500, (0, 130, 255))

    _put_text(canvas, "JOINT STATES", (1050, 650), (100, 210, 255), 0.65)
    joints = metrics.get("joint_states")
    joint_health = health["joint_states"]
    if joint_health.status == "missing":
        _put_column_text(canvas, "MISSING: glide model has no /joint_states",
                         (1050, 685), 430, (0, 100, 255), 0.46)
    elif joint_health.status == "fresh" and joints and joints.valid:
        v = joints.values
        _put_text(canvas, f"{v['count']} joints; position range {v['position_min_rad']:.3f}..{v['position_max_rad']:.3f} rad", (1050, 685), scale=0.46)
        yy = 712
        for name, position in v["joints"]:
            _put_column_text(canvas, dashboard_joint_line(name, position),
                             (1050, yy), 430, scale=0.46)
            yy += 23
        if v["remaining_count"]:
            _put_text(canvas, f"... and {v['remaining_count']} more", (1050, yy), scale=0.46)
    else:
        detail = joints.detail if joints else joint_health.detail
        _put_column_text(canvas, f"{joint_health.status.upper()}: {detail}",
                         (1050, 685), 430, (0, 130, 255), 0.46)
    _put_text(canvas, "q / Esc: quit" if _has_display() else "Headless snapshot mode", (20, height - 16), (130, 130, 130), 0.42)
    return canvas


def _parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    snapshots = parser.add_mutually_exclusive_group()
    snapshots.add_argument("--snapshot", metavar="PATH",
                           help="save PNG when all five core streams and RGB-D are fresh")
    snapshots.add_argument("--snapshot-joints", metavar="PATH",
                           help="save PNG when /joint_states is fresh; other streams may be missing")
    parser.add_argument("--timeout", type=float, default=120.0,
                        help="seconds to wait for startup/core sensor data (default: 120)")
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> int:
    args = _parse_args(argv)
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        print("--timeout must be a finite positive number", file=sys.stderr)
        return 2
    if dashboard_requires_display(args.snapshot, args.snapshot_joints) and not _has_display():
        print("No graphical display is available; use --snapshot PATH for headless capture.", file=sys.stderr)
        return 2

    import rclpy
    from cv_bridge import CvBridge, CvBridgeError
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import Image, Imu, JointState, LaserScan

    class SensorDashboard(Node):
        def __init__(self) -> None:
            super().__init__("g1_sensor_dashboard")
            self.bridge = CvBridge()
            self.frames = FrameSynchronizer(stale_after_sec=0.75)
            self.trackers = {name: StreamTracker(limit) for name, limit in STALE_LIMITS.items()}
            self.metrics: dict[str, MetricResult] = {}
            self.latest_rgb: Optional[np.ndarray] = None
            self.latest_depth: Optional[np.ndarray] = None
            qos = rclpy.qos.qos_profile_sensor_data
            self.create_subscription(Image, "/camera/image", self._on_rgb, qos)
            self.create_subscription(Image, "/camera/depth_image", self._on_depth, qos)
            self.create_subscription(LaserScan, "/scan", self._on_scan, qos)
            self.create_subscription(Imu, "/imu", self._on_imu, qos)
            self.create_subscription(Odometry, "/odom", self._on_odom, qos)
            self.create_subscription(JointState, "/joint_states", self._on_joints, qos)

        def _record(self, key: str, message: Any, result: MetricResult) -> None:
            now = time.monotonic()
            stamp = _header_stamp(message)
            self.trackers[key].update(
                now, stamp,
                valid=result.valid and stamp is not None,
                detail=result.detail if stamp is not None else "missing ROS header timestamp",
            )
            self.metrics[key] = result

        def _conversion_error(self, key: str, message: Any, reason: str) -> None:
            self.trackers[key].mark_invalid(time.monotonic(), reason, _header_stamp(message))
            self.metrics[key] = MetricResult(False, reason, {})

        def _on_rgb(self, message: Image) -> None:
            try:
                image = self.bridge.imgmsg_to_cv2(message, "bgr8")
                if image.ndim != 3 or image.shape[2] != 3 or image.size == 0:
                    raise ValueError("RGB image has invalid shape")
                self.latest_rgb = image
                self.frames.add_rgb(image, stamp_sec=_header_stamp(message), received_monotonic=time.monotonic())
                stamp = _header_stamp(message)
                self.trackers["camera_rgb"].update(
                    time.monotonic(), stamp, valid=stamp is not None,
                    detail="missing ROS header timestamp",
                )
            except (CvBridgeError, ValueError, TypeError) as error:
                self.frames.mark_invalid(f"RGB conversion failed: {error}")
                self._conversion_error("camera_rgb", message, f"RGB conversion failed: {error}")

        def _on_depth(self, message: Image) -> None:
            try:
                raw = self.bridge.imgmsg_to_cv2(message, "passthrough")
                depth = depth_to_meters(raw, message.encoding)
                if depth.ndim != 2 or depth.size == 0 or not np.any(np.isfinite(depth) & (depth > 0.0)):
                    raise ValueError("depth image has no valid positive measurements")
                self.latest_depth = depth
                self.frames.add_depth(depth, stamp_sec=_header_stamp(message), received_monotonic=time.monotonic())
                stamp = _header_stamp(message)
                self.trackers["camera_depth"].update(
                    time.monotonic(), stamp, valid=stamp is not None,
                    detail="missing ROS header timestamp",
                )
            except (CvBridgeError, ValueError, TypeError) as error:
                self.frames.mark_invalid(f"depth conversion failed: {error}")
                self._conversion_error("camera_depth", message, f"depth conversion failed: {error}")

        def _on_scan(self, message: LaserScan) -> None:
            result = summarize_laser_scan(message)
            self._record("scan", message, result)

        def _on_imu(self, message: Imu) -> None:
            self._record("imu", message, summarize_imu(message))

        def _on_odom(self, message: Odometry) -> None:
            self._record("odom", message, summarize_odometry(message))

        def _on_joints(self, message: JointState) -> None:
            self._record("joint_states", message, summarize_joint_state(message))

        def view(self, now: float) -> tuple[np.ndarray, dict[str, StreamHealth], str]:
            health = {key: tracker.snapshot(now) for key, tracker in self.trackers.items()}
            frame = self.frames.snapshot(now)
            frame_status = frame.status.upper()
            rgb = frame.pair.rgb.payload if frame.is_valid else None
            depth = frame.pair.depth.payload if frame.is_valid else None
            return render_dashboard(now, health, rgb, depth, frame_status, self.metrics), health, frame.status

    rclpy.init()
    node = SensorDashboard()
    started = time.monotonic()
    result_code = 0
    snapshot_argument = args.snapshot or args.snapshot_joints
    snapshot_path = os.path.abspath(snapshot_argument) if snapshot_argument else None
    joints_snapshot = bool(args.snapshot_joints)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.025)
            now = time.monotonic()
            panel, health, frame_status = node.view(now)
            core_fresh = all(health[key].status == "fresh" for key in CORE_STREAMS)
            frame_fresh = frame_status == "valid"
            if snapshot_path:
                snapshot_ready = (
                    health["joint_states"].status == "fresh"
                    if joints_snapshot
                    else core_fresh and frame_fresh
                )
                if snapshot_ready:
                    try:
                        os.makedirs(os.path.dirname(snapshot_path), exist_ok=True)
                        encoded_ok, png_data = cv2.imencode(".png", panel)
                        if encoded_ok:
                            png_data.tofile(snapshot_path)
                    except (OSError, cv2.error) as error:
                        print(f"Could not write snapshot PNG: {error}", file=sys.stderr)
                        result_code = 2
                        break
                    if not encoded_ok:
                        print(f"Could not write snapshot PNG: {snapshot_path}", file=sys.stderr)
                        result_code = 2
                    else:
                        print(f"Saved live sensor dashboard snapshot: {snapshot_path}")
                    break
                if now - started >= args.timeout:
                    if joints_snapshot:
                        summary = f"joint_states={health['joint_states'].status}"
                        requirement = "fresh /joint_states"
                    else:
                        summary = ", ".join(f"{key}={health[key].status}" for key in CORE_STREAMS)
                        requirement = f"fresh core streams/RGB-D pair ({frame_status})"
                    print(f"Timed out waiting for {requirement}: {summary}", file=sys.stderr)
                    result_code = 1
                    break
            else:
                try:
                    cv2.imshow(WINDOW_NAME, panel)
                    key = cv2.waitKey(1) & 0xFF
                except cv2.error as error:
                    print(f"OpenCV could not open a graphical window: {error}", file=sys.stderr)
                    result_code = 2
                    break
                if key in (ord("q"), 27):
                    break
                if now - started >= args.timeout and not core_fresh:
                    print("Timed out waiting for fresh core sensor streams.", file=sys.stderr)
                    result_code = 1
                    break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if not snapshot_path:
            cv2.destroyAllWindows()
    return result_code


if __name__ == "__main__":
    raise SystemExit(main())
