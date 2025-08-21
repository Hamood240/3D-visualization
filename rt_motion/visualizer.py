from __future__ import annotations
import threading
import time
from queue import Queue, Empty
from typing import Optional
import numpy as np

from .models import ProcessedFrame


class Visualizer(threading.Thread):
    """Visualizer that consumes ProcessedFrame objects and renders VPython scene.

    In headless mode, it prints periodic stats instead of rendering.
    """

    def __init__(self, frame_queue: Queue, target_fps: int = 60, headless: bool = False) -> None:
        super().__init__(name="Visualizer", daemon=True)
        self._q = frame_queue
        self._target_dt = 1.0 / float(target_fps)
        self._headless = bool(headless)
        self._stop = threading.Event()

        self._last_print_s = 0.0

        # VPython placeholders
        self._scene = None
        self._phone_box = None
        self._label = None

    def stop(self) -> None:
        self._stop.set()

    def _setup_vpython(self) -> None:
        from vpython import canvas, vector, box, color, label
        self._scene = canvas(title="IMU Motion", width=800, height=600, center=vector(0, 0, 0))
        self._phone_box = box(pos=vector(0, 0, 0), length=2, height=1, width=0.1, color=color.cyan)
        self._label = label(pos=vector(0, 1.5, 0), text="", height=12, box=False)

    def _quat_to_axis_angle(self, q: np.ndarray) -> tuple[np.ndarray, float]:
        # Convert [w,x,y,z] to rotation axis and angle (radians)
        w, x, y, z = q
        angle = 2.0 * np.arccos(np.clip(w, -1.0, 1.0))
        s = np.sqrt(1 - w*w)
        if s < 1e-6:
            axis = np.array([1.0, 0.0, 0.0])
        else:
            axis = np.array([x, y, z]) / s
        return axis, angle

    def run(self) -> None:
        if not self._headless:
            try:
                self._setup_vpython()
            except Exception:
                # Fall back to headless if VPython is unavailable
                self._headless = True

        last_update_s = time.monotonic()
        latest: Optional[ProcessedFrame] = None

        while not self._stop.is_set():
            start = time.monotonic()
            # Drain queue to latest
            try:
                while True:
                    latest = self._q.get(timeout=self._target_dt)
                    # non-blocking drain
                    while True:
                        latest = self._q.get_nowait()
            except Empty:
                pass
            if latest is None:
                continue

            if self._headless:
                if start - self._last_print_s >= 1.0:
                    e = latest.orientation.euler_rpy_deg
                    m = latest.motion
                    print(
                        f"RPY(deg): {e[0]:6.1f} {e[1]:6.1f} {e[2]:6.1f} | "
                        f"Speed: {m.speed_mps:5.2f} m/s | Dist: {m.total_distance_m:7.3f} m | "
                        f"RotCum(deg): {m.cumulative_rotation_deg_rpy[2]:6.1f} yaw",
                        flush=True,
                    )
                    self._last_print_s = start
            else:
                from vpython import vector, rate
                q = latest.orientation.quat_wxyz
                axis, angle = self._quat_to_axis_angle(q)
                self._phone_box.axis = vector(axis[0], axis[1], axis[2])
                self._phone_box.up = vector(0, 0, 1)
                self._phone_box.rotate(angle=angle, axis=self._phone_box.axis)
                e = latest.orientation.euler_rpy_deg
                m = latest.motion
                self._label.text = (
                    f"Roll {e[0]:.1f} Pitch {e[1]:.1f} Yaw {e[2]:.1f}\n"
                    f"Speed {m.speed_mps:.2f} m/s  Dist {m.total_distance_m:.2f} m\n"
                    f"RotCum Yaw {m.cumulative_rotation_deg_rpy[2]:.1f} deg"
                )
                rate(int(1.0 / self._target_dt))

            # Sleep to target framerate
            elapsed = time.monotonic() - start
            to_sleep = self._target_dt - max(0.0, elapsed)
            if to_sleep > 0:
                time.sleep(to_sleep)