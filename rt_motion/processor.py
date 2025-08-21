from __future__ import annotations
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional
import numpy as np

from .config import DEFAULTS
from .filters import HighPassFilter, LowPassFilter, Kalman1D
from .fusion import MadgwickWrapper, quaternion_to_euler_rpy_deg, quaternion_to_rotation_matrix
from .models import SensorSample, OrientationState, MotionState, ProcessedFrame


def wrap_angle_delta_deg(current_deg: float, previous_deg: float) -> float:
    """Shortest signed delta in degrees, wrapped to [-180, 180]."""
    delta = current_deg - previous_deg
    delta = (delta + 180.0) % 360.0 - 180.0
    return delta


@dataclass
class ZUPTState:
    is_stationary: bool = True
    last_change_s: float = 0.0


class MotionProcessor:
    """Process IMU samples -> orientation + linear motion state.

    Pipeline per sample:
    - Sensor fusion (Madgwick) to get orientation quaternion
    - Rotate accelerometer to world frame, subtract gravity
    - High/low-pass filtering
    - ZUPT motion detection using gyro norm and accel std window
    - Integrate to velocity and position, enforce zero velocity when stationary
    - Track cumulative rotation via fused Euler deltas
    """

    def __init__(
        self,
        sample_rate_hz: float = DEFAULTS.process_rate_hz,
        gravity_mps2: float = DEFAULTS.gravity_mps2,
        acc_lowpass_alpha: float = DEFAULTS.acc_lowpass_alpha,
        acc_highpass_alpha: float = DEFAULTS.acc_highpass_alpha,
        zupt_acc_thresh: float = DEFAULTS.zupt_acc_thresh,
        zupt_gyro_thresh: float = DEFAULTS.zupt_gyro_thresh,
        zupt_min_duration_s: float = DEFAULTS.zupt_min_duration_s,
        kalman_use: bool = DEFAULTS.kalman_use,
        kalman_process_var: float = DEFAULTS.kalman_process_var,
        kalman_measure_var: float = DEFAULTS.kalman_measure_var,
    ) -> None:
        self.dt = 1.0 / float(sample_rate_hz)
        self.gravity_mps2 = float(gravity_mps2)

        self.fusion = MadgwickWrapper(sample_rate_hz=sample_rate_hz)

        self.acc_lp = LowPassFilter(acc_lowpass_alpha)
        self.acc_hp = HighPassFilter(acc_highpass_alpha)

        self.zupt_acc_thresh = float(zupt_acc_thresh)
        self.zupt_gyro_thresh = float(zupt_gyro_thresh)
        self.zupt_min_duration_s = float(zupt_min_duration_s)
        self.zupt_state = ZUPTState(is_stationary=True, last_change_s=time.monotonic())
        self.acc_window: Deque[np.ndarray] = deque(maxlen=max(3, int(0.5 / self.dt)))

        self.world_velocity = np.zeros(3, dtype=float)
        self.world_position = np.zeros(3, dtype=float)
        self.total_distance_m = 0.0

        self.last_euler_deg = np.zeros(3, dtype=float)
        self.cumulative_rotation_deg_rpy = np.zeros(3, dtype=float)

        self.kalman_use = bool(kalman_use)
        self.kalman_filters = [Kalman1D(Q=kalman_process_var, R=kalman_measure_var) for _ in range(3)]

    def _detect_stationary(self, acc_world_no_g: np.ndarray, gyro_rps: np.ndarray, now_s: float) -> bool:
        self.acc_window.append(acc_world_no_g)
        acc_std = np.std(np.vstack(self.acc_window), axis=0) if len(self.acc_window) > 3 else np.array([np.inf, np.inf, np.inf])
        gyro_norm = float(np.linalg.norm(gyro_rps))
        stationary = bool(
            gyro_norm < self.zupt_gyro_thresh and float(np.linalg.norm(acc_std)) < self.zupt_acc_thresh
        )
        if stationary != self.zupt_state.is_stationary:
            # Debounce: require min duration
            if now_s - self.zupt_state.last_change_s >= self.zupt_min_duration_s:
                self.zupt_state = ZUPTState(is_stationary=stationary, last_change_s=now_s)
        return self.zupt_state.is_stationary

    def process(self, sample: SensorSample) -> ProcessedFrame:
        # Orientation
        q_wxyz = self.fusion.update(sample.acc_mps2, sample.gyr_rps, sample.mag_uT)
        euler_deg = quaternion_to_euler_rpy_deg(q_wxyz)

        # Body->world rotation
        R_bw = quaternion_to_rotation_matrix(q_wxyz)
        acc_world = R_bw @ sample.acc_mps2

        # Remove gravity (assumed along +Z in world frame)
        gravity_vec = np.array([0.0, 0.0, self.gravity_mps2])
        acc_world_no_g = acc_world - gravity_vec

        # Optional filtering
        acc_world_lp = self.acc_lp.update(acc_world_no_g)
        acc_world_hp = self.acc_hp.update(acc_world_no_g)
        # Blend: here we simply use low-pass for integration stability
        acc_use = acc_world_lp

        # ZUPT detection
        now_s = sample.timestamp_s
        is_stationary = self._detect_stationary(acc_world_no_g, sample.gyr_rps, now_s)

        # Integrate
        if is_stationary:
            # Apply Kalman update pulling velocity to 0
            if self.kalman_use:
                for i in range(3):
                    self.kalman_filters[i].x = self.world_velocity[i]
                    self.kalman_filters[i].update(0.0)
                    self.world_velocity[i] = self.kalman_filters[i].x
            # Hard reset small velocities
            self.world_velocity *= 0.0
        else:
            # Predict step for Kalman
            if self.kalman_use:
                for kf in self.kalman_filters:
                    kf.predict()
            self.world_velocity = self.world_velocity + acc_use * self.dt
            self.world_position = self.world_position + self.world_velocity * self.dt

        speed_mps = float(np.linalg.norm(self.world_velocity))
        self.total_distance_m += speed_mps * self.dt

        # Cumulative rotation via fused Euler
        delta_euler = np.array([
            wrap_angle_delta_deg(euler_deg[0], self.last_euler_deg[0]),
            wrap_angle_delta_deg(euler_deg[1], self.last_euler_deg[1]),
            wrap_angle_delta_deg(euler_deg[2], self.last_euler_deg[2]),
        ])
        self.cumulative_rotation_deg_rpy += delta_euler
        self.last_euler_deg = euler_deg

        orientation = OrientationState(quat_wxyz=q_wxyz, euler_rpy_deg=euler_deg, timestamp_s=sample.timestamp_s)
        motion = MotionState(
            world_acc_no_g_mps2=acc_world_no_g,
            world_velocity_mps=self.world_velocity.copy(),
            world_position_m=self.world_position.copy(),
            speed_mps=speed_mps,
            total_distance_m=self.total_distance_m,
            cumulative_rotation_deg_rpy=self.cumulative_rotation_deg_rpy.copy(),
            timestamp_s=sample.timestamp_s,
        )
        return ProcessedFrame(sample=sample, orientation=orientation, motion=motion)