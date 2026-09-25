"""RGB-D viewer and capture utility for the G1 Gazebo camera topics.

The capture helpers in this module deliberately do not depend on ROS.  This
makes the file's data conversion and persistence behavior testable with plain
Python, NumPy, and OpenCV while keeping ROS imports inside ``main``.
"""

from __future__ import annotations

from datetime import datetime
import os
from pathlib import Path
import time
from typing import Optional

import cv2
import numpy as np

from .frame_sync import FrameSynchronizer


# g1_guide/ -> g1_guide/ -> src/ -> ws/ -> project root (symlink-install resolves to source)
DEFAULT_CAPTURE_DIR = os.environ.get(
    "G1_CAPTURE_DIR", str(Path(__file__).resolve().parents[4] / "captures"))
RGB_WINDOW = "G1 RGB"
DEPTH_WINDOW = "G1 Depth (colorized)"
DEPTH_DISPLAY_MAX_M = 10.0


def capture_directory() -> Path:
    """Return the configured capture directory without creating it."""

    return Path(os.environ.get("G1_CAPTURE_DIR", DEFAULT_CAPTURE_DIR)).expanduser()


def depth_to_meters(depth_image: np.ndarray, encoding: str = "") -> np.ndarray:
    """Convert a ROS depth image array to float32 meters.

    ``32FC1`` depth images are already meters.  ``16UC1`` images are the
    standard ROS millimeter representation.  The dtype fallback handles
    bridges that omit the encoding while preserving the same convention.
    Invalid and zero values remain invalid/zero for downstream masking.
    """

    depth = np.asarray(depth_image)
    normalized_encoding = encoding.lower()
    if normalized_encoding in {"16uc1", "mono16"} or depth.dtype == np.uint16:
        return depth.astype(np.float32) / 1000.0
    return depth.astype(np.float32, copy=False)


def center_depth_median(depth_meters: np.ndarray, window_size: int = 10) -> Optional[float]:
    """Return median valid depth in a centered ROI, not a target distance."""

    if depth_meters.ndim < 2 or depth_meters.size == 0:
        return None

    height, width = depth_meters.shape[:2]
    half = window_size // 2
    center_y, center_x = height // 2, width // 2
    region = depth_meters[
        max(0, center_y - half) : min(height, center_y + half),
        max(0, center_x - half) : min(width, center_x + half),
    ]
    valid = region[np.isfinite(region) & (region > 0.0)]
    return float(np.median(valid)) if valid.size else None


def depth_colormap(depth_meters: np.ndarray, max_distance_m: float = DEPTH_DISPLAY_MAX_M) -> np.ndarray:
    """Create a BGR pseudo-color image, with invalid pixels rendered black."""

    depth = np.asarray(depth_meters, dtype=np.float32)
    valid = np.isfinite(depth) & (depth > 0.0)
    clipped = np.clip(depth, 0.0, max_distance_m)
    grayscale = np.zeros(depth.shape, dtype=np.uint8)
    grayscale[valid] = np.asarray(
        clipped[valid] * (255.0 / max_distance_m), dtype=np.uint8
    )
    colored = cv2.applyColorMap(grayscale, cv2.COLORMAP_JET)
    colored[~valid] = 0
    return colored


def _capture_stamp() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S_%f")


def _message_stamp_seconds(message: object) -> float:
    """Extract a ROS message time stamp without importing ROS."""

    stamp = message.header.stamp
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def save_capture(
    rgb_image: np.ndarray,
    depth_meters: np.ndarray,
    output_directory: Path | str,
    timestamp: Optional[str] = None,
    *,
    frame_status: str = "valid",
) -> dict[str, Path]:
    """Save RGB, colorized depth, float meters, and uint16 millimeters.

    The returned mapping contains the exact paths written.  The raw depth
    NumPy file is always float32 meters; the PNG depth file is always uint16
    millimeters, with invalid/non-finite values written as zero.
    """

    if frame_status != "valid":
        raise ValueError(
            f"refusing to save RGB-D frame with status={frame_status!r}; "
            "RGB and depth must be a fresh synchronized pair"
        )

    output_path = Path(output_directory).expanduser()
    output_path.mkdir(parents=True, exist_ok=True)
    stamp = timestamp or _capture_stamp()

    rgb = np.asarray(rgb_image)
    depth = np.asarray(depth_meters, dtype=np.float32)
    if rgb.ndim != 3 or rgb.shape[2] != 3:
        raise ValueError("RGB image must have shape (height, width, 3)")
    if depth.ndim != 2:
        raise ValueError("Depth image must have shape (height, width)")

    valid_depth = np.isfinite(depth) & (depth > 0.0)
    depth_mm = np.zeros(depth.shape, dtype=np.uint16)
    depth_mm[valid_depth] = np.clip(
        np.rint(depth[valid_depth] * 1000.0), 0.0, np.iinfo(np.uint16).max
    ).astype(np.uint16)

    paths = {
        "rgb": output_path / f"rgb_{stamp}.png",
        "depth_color": output_path / f"depth_color_{stamp}.png",
        "depth_meters": output_path / f"depth_m_{stamp}.npy",
        "depth_millimeters": output_path / f"depth_mm_{stamp}.png",
    }
    if not cv2.imwrite(str(paths["rgb"]), rgb):
        raise IOError(f"failed to write {paths['rgb']}")
    if not cv2.imwrite(str(paths["depth_color"]), depth_colormap(depth)):
        raise IOError(f"failed to write {paths['depth_color']}")
    np.save(paths["depth_meters"], depth.astype(np.float32, copy=False))
    if not cv2.imwrite(str(paths["depth_millimeters"]), depth_mm):
        raise IOError(f"failed to write {paths['depth_millimeters']}")
    return paths


def main() -> None:
    """Run the ROS 2 viewer until ``q`` or Ctrl-C is received."""

    import rclpy
    from cv_bridge import CvBridge, CvBridgeError
    from rclpy.node import Node
    from sensor_msgs.msg import Image

    class CameraViewer(Node):
        def __init__(self) -> None:
            super().__init__("g1_camera_viewer")
            self.bridge = CvBridge()
            self.frames = FrameSynchronizer()
            self.last_report_time = 0.0
            self.capture_dir = capture_directory()
            sensor_qos = rclpy.qos.qos_profile_sensor_data
            self.create_subscription(
                Image, "/camera/image", self._on_rgb, sensor_qos
            )
            self.create_subscription(
                Image, "/camera/depth_image", self._on_depth, sensor_qos
            )
            self.get_logger().info(f"Saving captures to {self.capture_dir}")

        def _on_rgb(self, message: Image) -> None:
            try:
                image = self.bridge.imgmsg_to_cv2(message, "bgr8")
                self.frames.add_rgb(
                    image,
                    stamp_sec=_message_stamp_seconds(message),
                    received_monotonic=time.monotonic(),
                )
            except CvBridgeError as error:
                self.frames.mark_invalid(f"RGB conversion failed: {error}")
                self.get_logger().warning(f"RGB conversion failed: {error}")

        def _on_depth(self, message: Image) -> None:
            try:
                raw_depth = self.bridge.imgmsg_to_cv2(message, "passthrough")
                self.frames.add_depth(
                    depth_to_meters(raw_depth, message.encoding),
                    stamp_sec=_message_stamp_seconds(message),
                    received_monotonic=time.monotonic(),
                )
            except CvBridgeError as error:
                self.frames.mark_invalid(f"depth conversion failed: {error}")
                self.get_logger().warning(f"Depth conversion failed: {error}")

        @staticmethod
        def _status_image(text: str, shape: tuple[int, int, int]) -> np.ndarray:
            image = np.zeros(shape, dtype=np.uint8)
            cv2.putText(
                image,
                text[:70],
                (20, max(30, shape[0] // 2)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 180, 255),
                2,
                cv2.LINE_AA,
            )
            return image

        def show_frame(self) -> None:
            now = time.monotonic()
            snapshot = self.frames.snapshot(now)
            if snapshot.is_valid:
                rgb_image = snapshot.pair.rgb.payload
                depth_meters = snapshot.pair.depth.payload
                cv2.imshow(RGB_WINDOW, rgb_image)
                cv2.imshow(DEPTH_WINDOW, depth_colormap(depth_meters))
            else:
                status = f"{snapshot.status.upper()}: {snapshot.message}"
                cv2.imshow(RGB_WINDOW, self._status_image(status, (480, 640, 3)))
                cv2.imshow(DEPTH_WINDOW, self._status_image(status, (480, 640, 3)))

            if now - self.last_report_time >= 0.5:
                if not snapshot.is_valid:
                    self.get_logger().info(
                        f"RGB-D status: {snapshot.status} ({snapshot.message})"
                    )
                else:
                    median = center_depth_median(snapshot.pair.depth.payload)
                    if median is None:
                        self.get_logger().info(
                            "center ROI depth (10x10; not target distance): invalid"
                        )
                    else:
                        self.get_logger().info(
                            "center ROI depth (10x10; not target distance): "
                            f"{median:.3f} m"
                        )
                self.last_report_time = now

        def save_current_frame(self) -> None:
            snapshot = self.frames.snapshot(time.monotonic())
            if not snapshot.is_valid:
                self.get_logger().warning(
                    f"Cannot save RGB-D frame: {snapshot.status} ({snapshot.message})"
                )
                return
            try:
                paths = save_capture(
                    snapshot.pair.rgb.payload,
                    snapshot.pair.depth.payload,
                    self.capture_dir,
                    frame_status=snapshot.status,
                )
            except (OSError, ValueError, IOError) as error:
                self.get_logger().error(f"Capture failed: {error}")
                return
            self.get_logger().info(
                "Saved RGB-D capture: " + ", ".join(str(path) for path in paths.values())
            )

    rclpy.init()
    node = CameraViewer()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.03)
            node.show_frame()
            key = cv2.waitKey(1) & 0xFF
            if key == ord("s"):
                node.save_current_frame()
            elif key == ord("q"):
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
