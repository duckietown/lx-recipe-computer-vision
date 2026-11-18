"""Physical parameters of the robot (wheel baseline, wheel radius, encoder resolution).

ROS 2 has no global parameter server, so these live in a calibration file on the robot
(``/data/config/calibrations/robot/<robot>.yaml``, falling back to ``default.yaml``). The ``dt_robot_utils``
helper reads it on recent images; this module works on older images too and finally falls back to the
Duckiebot DB21 defaults.
"""
import os
from dataclasses import dataclass
from typing import Optional

import yaml

DEFAULT_WHEEL_BASELINE = 0.1      # [m]  distance between the wheels
DEFAULT_WHEEL_RADIUS = 0.0318     # [m]
DEFAULT_ENCODER_RESOLUTION = 135  # [ticks / revolution]
CALIBRATION_DIR = "/data/config/calibrations/robot"


@dataclass
class WheelParams:
    baseline: float = DEFAULT_WHEEL_BASELINE
    radius: float = DEFAULT_WHEEL_RADIUS
    encoder_resolution: int = DEFAULT_ENCODER_RESOLUTION
    source: str = "defaults"


def _from_file(path: str) -> Optional[WheelParams]:
    if not os.path.isfile(path):
        return None
    with open(path, "r") as f:
        data = yaml.safe_load(f) or {}
    return WheelParams(
        baseline=float(data.get("wheel_baseline", DEFAULT_WHEEL_BASELINE)),
        radius=float(data.get("wheel_radius", DEFAULT_WHEEL_RADIUS)),
        encoder_resolution=int(data.get("encoder_resolution", DEFAULT_ENCODER_RESOLUTION)),
        source=path,
    )


def get_wheel_params(robot_name: Optional[str] = None) -> WheelParams:
    """Return the wheel parameters of this robot, trying (in order) dt_robot_utils, the per-robot
    calibration file, the default calibration file and the built-in defaults."""
    try:
        from dt_robot_utils import get_robot_params  # type: ignore

        rp = get_robot_params()
        return WheelParams(
            baseline=float(rp.wheel_baseline),
            radius=float(rp.wheel_radius),
            encoder_resolution=int(getattr(rp, "encoder_resolution", DEFAULT_ENCODER_RESOLUTION)),
            source="dt_robot_utils",
        )
    except Exception:  # noqa: BLE001 - helper missing on older images, or no file
        pass
    robot_name = robot_name or os.environ.get("VEHICLE_NAME", "")
    for candidate in ([os.path.join(CALIBRATION_DIR, f"{robot_name}.yaml")] if robot_name else []) + [
        os.path.join(CALIBRATION_DIR, "default.yaml")
    ]:
        params = _from_file(candidate)
        if params is not None:
            return params
    return WheelParams()
