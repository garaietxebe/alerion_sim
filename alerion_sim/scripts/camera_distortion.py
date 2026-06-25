#!/usr/bin/env python3
"""
Brown-Conrady lens distortion post-processor.

Gazebo Harmonic's Ogre2 renderer silently ignores the SDF distortion element.
This node applies the same Brown-Conrady (Plumb Bob) model as a ROS 2
post-processing step using pure numpy remapping.
"""

from typing import Any

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

# ---------------------------------------------------------------------------
# ROS Image <-> numpy conversion (no cv_bridge or cv2)
# ---------------------------------------------------------------------------

_ENCODING_CHANNELS = {
    "rgb8": 3,
    "bgr8": 3,
    "rgba8": 4,
    "bgra8": 4,
    "mono8": 1,
    "8UC1": 1,
    "mono16": 1,
    "16UC1": 1,
}


def _imgmsg_to_numpy(msg: Image) -> np.ndarray:
    channels = _ENCODING_CHANNELS.get(msg.encoding, 3)
    dtype = np.uint16 if "16" in msg.encoding else np.uint8
    arr = np.frombuffer(bytes(msg.data), dtype=dtype)
    if channels == 1:
        return arr.reshape(msg.height, msg.width)
    return arr.reshape(msg.height, msg.width, channels)


def _numpy_to_imgmsg(arr: np.ndarray, encoding: str, header: Any) -> Image:
    msg = Image()
    msg.header = header
    msg.height = arr.shape[0]
    msg.width = arr.shape[1]
    msg.encoding = encoding
    channels = arr.shape[2] if arr.ndim == 3 else 1
    msg.step = arr.shape[1] * channels * arr.dtype.itemsize
    msg.data = arr.tobytes()
    return msg


# ---------------------------------------------------------------------------
# Iterative undistort and bilinear remap (no cv2)
# ---------------------------------------------------------------------------


def _undistort_points(pts: np.ndarray, K: np.ndarray, D: np.ndarray) -> np.ndarray:
    """
    Iteratively invert the Brown-Conrady model (equivalent to cv2.undistortPoints with P=K).

    pts : (N, 2)  distorted pixel coordinates
    K   : (3, 3)  intrinsic matrix
    D   : (5,)    [k1, k2, p1, p2, k3]
    Returns (N, 2) undistorted pixel coordinates.
    """
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    k1, k2, p1, p2, k3 = D

    x = (pts[:, 0] - cx) / fx
    y = (pts[:, 1] - cy) / fy

    # 5 iterations are sufficient for convergence
    x0, y0 = x.copy(), y.copy()
    for _ in range(5):
        r2 = x * x + y * y
        k_rad = 1.0 + k1 * r2 + k2 * r2**2 + k3 * r2**3
        delta_x = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
        delta_y = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
        x = (x0 - delta_x) / k_rad
        y = (y0 - delta_y) / k_rad

    return np.stack([x * fx + cx, y * fy + cy], axis=-1)


def _remap_bilinear(src: np.ndarray, map_x: np.ndarray, map_y: np.ndarray) -> np.ndarray:
    """Bilinear interpolation with BORDER_CONSTANT=0 outside the image boundary."""
    h, w = src.shape[:2]

    x0 = np.floor(map_x).astype(np.int32)
    y0 = np.floor(map_y).astype(np.int32)
    x1 = x0 + 1
    y1 = y0 + 1

    wx = (map_x - x0).astype(np.float32)
    wy = (map_y - y0).astype(np.float32)

    valid = (map_x >= 0) & (map_x < w) & (map_y >= 0) & (map_y < h)

    x0c = np.clip(x0, 0, w - 1)
    x1c = np.clip(x1, 0, w - 1)
    y0c = np.clip(y0, 0, h - 1)
    y1c = np.clip(y1, 0, h - 1)

    if src.ndim == 3:
        wx = wx[..., np.newaxis]
        wy = wy[..., np.newaxis]

    p00 = src[y0c, x0c].astype(np.float32)
    p01 = src[y0c, x1c].astype(np.float32)
    p10 = src[y1c, x0c].astype(np.float32)
    p11 = src[y1c, x1c].astype(np.float32)

    result = ((1 - wy) * ((1 - wx) * p00 + wx * p01) + wy * ((1 - wx) * p10 + wx * p11)).astype(
        src.dtype
    )

    if src.ndim == 3:
        result[~valid] = 0
    else:
        result[~valid] = 0

    return result


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------


class CameraDistortionNode(Node):
    def __init__(self) -> None:
        super().__init__("camera_distortion")

        self.declare_parameter("k1", -0.45)
        self.declare_parameter("k2", 0.18)
        self.declare_parameter("k3", 0.0)
        self.declare_parameter("p1", 0.003)
        self.declare_parameter("p2", -0.002)
        self.declare_parameter("input_topic", "/camera/image_raw")
        self.declare_parameter("output_topic", "/camera/image_distorted")
        self.declare_parameter("info_topic", "")

        p = self.get_parameter
        k1 = p("k1").value
        k2 = p("k2").value
        k3 = p("k3").value
        p1 = p("p1").value
        p2 = p("p2").value
        self._D = np.array([k1, k2, p1, p2, k3], dtype=np.float64)

        in_topic = p("input_topic").value
        out_topic = p("output_topic").value
        info_topic = p("info_topic").value or f"{in_topic}/camera_info"

        # remap tables built once from the first CameraInfo message
        self._map_x: np.ndarray | None = None
        self._map_y: np.ndarray | None = None

        self._pub = self.create_publisher(Image, out_topic, 10)
        self.create_subscription(CameraInfo, info_topic, self._info_cb, 1)
        self.create_subscription(Image, in_topic, self._image_cb, 10)

        self.get_logger().info(
            f"Brown-Conrady distortion active: "
            f"k1={k1:.3f} k2={k2:.3f} k3={k3:.3f} p1={p1:.4f} p2={p2:.4f}"
        )

    def _info_cb(self, msg: CameraInfo) -> None:
        if self._map_x is not None:
            return

        h, w = msg.height, msg.width
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)

        xs, ys = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))
        pts_flat = np.stack([xs.ravel(), ys.ravel()], axis=-1).astype(np.float64)

        undist = _undistort_points(pts_flat, K, self._D).astype(np.float32)
        self._map_x = undist[:, 0].reshape(h, w)
        self._map_y = undist[:, 1].reshape(h, w)

        self.get_logger().debug(f"Remap tables built: {w}x{h} fx={K[0, 0]:.1f} fy={K[1, 1]:.1f}")

    def _image_cb(self, msg: Image) -> None:
        if self._map_x is None or self._map_y is None:
            return

        img = _imgmsg_to_numpy(msg)
        distorted = _remap_bilinear(img, self._map_x, self._map_y)
        self._pub.publish(_numpy_to_imgmsg(distorted, msg.encoding, msg.header))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CameraDistortionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
