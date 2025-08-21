from __future__ import annotations
import numpy as np
from ahrs.filters import Madgwick


def quaternion_normalize(q_wxyz: np.ndarray) -> np.ndarray:
    q = np.array(q_wxyz, dtype=float)
    n = np.linalg.norm(q)
    if n == 0:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / n


def quaternion_to_euler_rpy_deg(q_wxyz: np.ndarray) -> np.ndarray:
    """Convert quaternion [w, x, y, z] to roll, pitch, yaw in degrees.
    Uses aerospace sequence ZYX (yaw-pitch-roll).
    """
    w, x, y, z = quaternion_normalize(q_wxyz)

    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = np.degrees(np.arctan2(sinr_cosp, cosr_cosp))

    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = np.degrees(np.sign(sinp) * (np.pi / 2))
    else:
        pitch = np.degrees(np.arcsin(sinp))

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.degrees(np.arctan2(siny_cosp, cosy_cosp))

    return np.array([roll, pitch, yaw], dtype=float)


def quaternion_to_rotation_matrix(q_wxyz: np.ndarray) -> np.ndarray:
    """Return 3x3 rotation matrix from body to world (ZYX convention)."""
    w, x, y, z = quaternion_normalize(q_wxyz)
    R = np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),       2*(x*z + y*w)],
        [2*(x*y + z*w),         1 - 2*(x*x + z*z),   2*(y*z - x*w)],
        [2*(x*z - y*w),         2*(y*z + x*w),       1 - 2*(x*x + y*y)],
    ], dtype=float)
    return R


class MadgwickWrapper:
    """Thin wrapper around ahrs.filters.Madgwick for clarity."""

    def __init__(self, sample_rate_hz: float = 100.0, beta: float = 0.1) -> None:
        self.sample_period = 1.0 / float(sample_rate_hz)
        self.filter = Madgwick(sampleperiod=self.sample_period, beta=beta)
        self.quaternion_wxyz = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)

    def update(self, acc_mps2: np.ndarray, gyr_rps: np.ndarray, mag_uT: np.ndarray | None = None) -> np.ndarray:
        if mag_uT is None:
            q = self.filter.updateIMU(self.quaternion_wxyz, gyr_rps, acc_mps2)
        else:
            q = self.filter.updateMARG(self.quaternion_wxyz, gyr_rps, acc_mps2, mag_uT)
        if q is not None:
            self.quaternion_wxyz = quaternion_normalize(q)
        return self.quaternion_wxyz