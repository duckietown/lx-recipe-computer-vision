"""Conversions between ``sensor_msgs/CompressedImage`` and numpy BGR images (no cv_bridge)."""
from typing import Optional

import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header

try:  # fast path when PyTurboJPEG is installed (it is in the Duckietown images)
    from turbojpeg import TurboJPEG

    _JPEG: Optional["TurboJPEG"] = TurboJPEG()
except Exception:  # noqa: BLE001
    _JPEG = None


def compressed_to_bgr(msg: CompressedImage) -> np.ndarray:
    """Decode a JPEG/PNG ``CompressedImage`` into an ``HxWx3`` BGR uint8 array."""
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if _JPEG is not None and "jpeg" in (msg.format or "jpeg").lower():
        try:
            return _JPEG.decode(buf)
        except Exception:  # noqa: BLE001 - fall back to OpenCV
            pass
    img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError("could not decode CompressedImage (format=%r, %d bytes)" % (msg.format, len(buf)))
    return img


def compressed_to_rgb(msg: CompressedImage) -> np.ndarray:
    """Same as :func:`compressed_to_bgr` but returns RGB."""
    return cv2.cvtColor(compressed_to_bgr(msg), cv2.COLOR_BGR2RGB)


def bgr_to_compressed(img: np.ndarray, header: Optional[Header] = None, quality: int = 80) -> CompressedImage:
    """Encode a BGR (or grayscale) uint8 array into a JPEG ``CompressedImage``."""
    if img.ndim == 2:
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    if _JPEG is not None:
        data = _JPEG.encode(np.ascontiguousarray(img), quality=quality)
    else:
        ok, enc = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
        if not ok:
            raise ValueError("could not encode image")
        data = enc.tobytes()
    msg = CompressedImage()
    if header is not None:
        msg.header = header
    msg.format = "jpeg"
    msg.data = data
    return msg


def rgb_to_compressed(img: np.ndarray, header: Optional[Header] = None, quality: int = 80) -> CompressedImage:
    """Encode an RGB uint8 array into a JPEG ``CompressedImage``."""
    return bgr_to_compressed(cv2.cvtColor(img, cv2.COLOR_RGB2BGR), header, quality)
