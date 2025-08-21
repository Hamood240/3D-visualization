from dataclasses import dataclass


@dataclass
class Defaults:
    udp_ip: str = "0.0.0.0"
    udp_port: int = 5555
    sample_rate_hz: float = 100.0
    process_rate_hz: float = 100.0
    gravity_mps2: float = 9.80665

    # Filtering
    acc_lowpass_alpha: float = 0.1
    acc_highpass_alpha: float = 0.01

    # ZUPT/motion detection
    zupt_acc_thresh: float = 0.15  # m/s^2 std threshold for stationary
    zupt_gyro_thresh: float = 0.02  # rad/s norm threshold for stationary
    zupt_min_duration_s: float = 0.2

    # Kalman (1D per axis velocity)
    kalman_use: bool = False
    kalman_process_var: float = 0.5
    kalman_measure_var: float = 2.0

    # Visualization
    target_fps: int = 60
    headless: bool = False


DEFAULTS = Defaults()