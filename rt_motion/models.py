from __future__ import annotations
from dataclasses import dataclass
from typing import Optional, Tuple
import numpy as np


@dataclass
class SensorSample:
    """Container for a single IMU sample in SI units.

    - acc_mps2: numpy array (3,) accelerometer in m/s^2
    - gyr_rps: numpy array (3,) gyroscope in rad/s
    - mag_uT: numpy array (3,) magnetometer in microtesla
    - timestamp_s: float, seconds (monotonic)
    """
    acc_mps2: np.ndarray
    gyr_rps: np.ndarray
    mag_uT: np.ndarray
    timestamp_s: float


@dataclass
class OrientationState:
    """Orientation result from sensor fusion.

    - quat_wxyz: numpy array (4,) quaternion [w, x, y, z]
    - euler_rpy_deg: numpy array (3,) [roll, pitch, yaw] in degrees
    - timestamp_s: time the orientation corresponds to
    """
    quat_wxyz: np.ndarray
    euler_rpy_deg: np.ndarray
    timestamp_s: float


@dataclass
class MotionState:
    """Linear motion state in world frame.

    - world_acc_no_g_mps2: numpy array (3,)
    - world_velocity_mps: numpy array (3,)
    - world_position_m: numpy array (3,)
    - speed_mps: float, instantaneous speed
    - total_distance_m: float, cumulative path length
    - cumulative_rotation_deg_rpy: numpy array (3,) integrated rotation [roll, pitch, yaw]
    - timestamp_s: float
    """
    world_acc_no_g_mps2: np.ndarray
    world_velocity_mps: np.ndarray
    world_position_m: np.ndarray
    speed_mps: float
    total_distance_m: float
    cumulative_rotation_deg_rpy: np.ndarray
    timestamp_s: float


@dataclass
class ProcessedFrame:
    """Bundle of processed outputs for visualization and logging."""
    sample: SensorSample
    orientation: OrientationState
    motion: MotionState