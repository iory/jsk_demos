"""cv_bridge-free helpers for ``sensor_msgs/Image`` and ``CompressedImage``.

Why this exists
---------------
``cv_bridge`` ships as a C extension built against a single Python ABI
(e.g. ``cv_bridge_boost.cpython-38-...so`` on Ubuntu 20.04 / ROS noetic).
When the node runs under a different Python (e.g. a uv-managed Python 3.10),
that extension cannot be loaded and ``from cv_bridge import CvBridge``
fails at import time.

For our use case (BGR8 frames in / annotated BGR8 frames out) the bridge
is doing nothing more than ``np.frombuffer + reshape + cv2.cvtColor``,
so we re-implement that here. Compressed images are decoded via
``cv2.imdecode``.
"""

import numpy as np

import cv2
import sensor_msgs.msg


_ENCODING_INFO = {
    "bgr8":  (np.uint8, 3),
    "rgb8":  (np.uint8, 3),
    "bgra8": (np.uint8, 4),
    "rgba8": (np.uint8, 4),
    "mono8": (np.uint8, 1),
    "8UC1":  (np.uint8, 1),
    "8UC3":  (np.uint8, 3),
    "8UC4":  (np.uint8, 4),
    "mono16": (np.uint16, 1),
    "16UC1": (np.uint16, 1),
    "32FC1": (np.float32, 1),
}


def _to_bgr8(arr, src_encoding):
    """Convert ``arr`` to a 3-channel BGR8 numpy image."""
    if src_encoding == "bgr8":
        return arr
    if src_encoding == "rgb8":
        return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
    if src_encoding == "bgra8":
        return cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)
    if src_encoding == "rgba8":
        return cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR)
    if src_encoding in ("mono8", "8UC1"):
        return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
    if src_encoding == "8UC3":
        # Unknown channel order — assume BGR.
        return arr
    if src_encoding == "8UC4":
        return cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)
    raise ValueError(
        "Cannot convert encoding {!r} to bgr8".format(src_encoding)
    )


def imgmsg_to_cv2(msg, desired_encoding="bgr8"):
    """Decode a raw ``sensor_msgs/Image`` into a numpy array.

    Parameters
    ----------
    msg : sensor_msgs.msg.Image
    desired_encoding : str
        Either ``"passthrough"`` (return whatever the source encoding is)
        or ``"bgr8"`` (the only conversion currently implemented).

    Returns
    -------
    numpy.ndarray
        2D for single-channel images, 3D ``(H, W, C)`` otherwise.
    """
    if msg.encoding not in _ENCODING_INFO:
        raise ValueError(
            "Unsupported sensor_msgs/Image encoding: {!r}".format(msg.encoding)
        )
    dtype, channels = _ENCODING_INFO[msg.encoding]
    item_size = np.dtype(dtype).itemsize
    expected_step = msg.width * channels * item_size

    raw = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.step == expected_step:
        arr = raw.view(dtype).reshape(msg.height, msg.width, channels)
    else:
        # The publisher inserted row padding (``step > width * bpp``).
        arr = (
            raw.reshape(msg.height, msg.step)[:, :expected_step]
            .reshape(-1)
            .view(dtype)
            .reshape(msg.height, msg.width, channels)
        )

    if msg.is_bigendian and item_size > 1:
        arr = arr.byteswap()

    if channels == 1:
        arr = arr[:, :, 0]

    if desired_encoding == "passthrough" or desired_encoding == msg.encoding:
        return arr.copy()
    if desired_encoding == "bgr8":
        return _to_bgr8(np.ascontiguousarray(arr), msg.encoding)
    raise ValueError(
        "Unsupported desired_encoding: {!r}".format(desired_encoding)
    )


def compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8"):
    """Decode a ``sensor_msgs/CompressedImage`` (JPEG / PNG) into BGR8.

    Depth (``compressedDepth``) is not handled here.
    """
    arr = np.frombuffer(msg.data, dtype=np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if img is None:
        raise RuntimeError(
            "cv2.imdecode failed for CompressedImage (format={!r})".format(msg.format)
        )
    fmt = (msg.format or "").lower()
    if "rgb" in fmt and "bgr" not in fmt:
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    if desired_encoding in ("passthrough", "bgr8"):
        return img
    raise ValueError(
        "Unsupported desired_encoding for CompressedImage: {!r}".format(desired_encoding)
    )


def cv2_to_imgmsg(arr, encoding="bgr8"):
    """Build a ``sensor_msgs/Image`` from a numpy array.

    Only BGR8 is implemented (caller is expected to convert beforehand).
    """
    if encoding != "bgr8":
        raise ValueError(
            "Only encoding='bgr8' is implemented (got {!r})".format(encoding)
        )
    if arr.dtype != np.uint8 or arr.ndim != 3 or arr.shape[2] != 3:
        raise ValueError(
            "Expected an (H, W, 3) uint8 BGR array, got shape={} dtype={}".format(
                arr.shape, arr.dtype
            )
        )
    arr = np.ascontiguousarray(arr)
    msg = sensor_msgs.msg.Image()
    msg.height = int(arr.shape[0])
    msg.width = int(arr.shape[1])
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    msg.step = int(arr.shape[1] * 3)
    msg.data = arr.tobytes()
    return msg
