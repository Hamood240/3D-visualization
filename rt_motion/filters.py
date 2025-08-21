from __future__ import annotations
from dataclasses import dataclass
import numpy as np


class LowPassFilter:
    """Exponential low-pass filter y_t = alpha*x_t + (1-alpha)*y_{t-1}."""

    def __init__(self, alpha: float, initial: float | np.ndarray | None = None) -> None:
        self.alpha = float(alpha)
        self.y = None if initial is None else np.array(initial, dtype=float)

    def update(self, x: float | np.ndarray) -> np.ndarray:
        x = np.array(x, dtype=float)
        if self.y is None:
            self.y = x.copy()
        else:
            self.y = self.alpha * x + (1.0 - self.alpha) * self.y
        return self.y


class HighPassFilter:
    """Simple high-pass via x - lowpass(x)."""

    def __init__(self, alpha_lowpass: float) -> None:
        self.lp = LowPassFilter(alpha_lowpass)

    def update(self, x: float | np.ndarray) -> np.ndarray:
        x = np.array(x, dtype=float)
        return x - self.lp.update(x)


class ComplementaryFilter:
    """Blend two signals: y = a*x_fast + (1-a)*x_slow."""

    def __init__(self, alpha_fast: float) -> None:
        self.alpha_fast = float(alpha_fast)

    def update(self, x_fast: np.ndarray, x_slow: np.ndarray) -> np.ndarray:
        return self.alpha_fast * np.array(x_fast) + (1.0 - self.alpha_fast) * np.array(x_slow)


@dataclass
class Kalman1D:
    """Very simple 1D Kalman filter for velocity stabilization.

    State: x (scalar), covariance P
    Model: x_k = x_{k-1}
    Process noise: Q
    Measurement: z (e.g., 0 during ZUPT), measurement noise R
    """

    x: float = 0.0
    P: float = 1.0
    Q: float = 0.5
    R: float = 2.0

    def predict(self) -> None:
        self.P = self.P + self.Q

    def update(self, z: float) -> None:
        # K = P / (P + R)
        K = self.P / (self.P + self.R)
        self.x = self.x + K * (z - self.x)
        self.P = (1.0 - K) * self.P