import socket
import threading
import time
from typing import Optional, Callable
import numpy as np

from .models import SensorSample


class UDPReceiver(threading.Thread):
    """Background thread to receive UDP CSV from HyperIMU and emit SensorSample objects.

    The expected CSV should contain at least 9 numeric fields corresponding to
    accelerometer (m/s^2), gyroscope (rad/s), and magnetometer (µT):
    ax, ay, az, gx, gy, gz, mx, my, mz, [optional timestamp]

    Any additional fields are ignored. If a timestamp is present and numeric, it
    is ignored by default; we prefer monotonic clock here.
    """

    def __init__(
        self,
        ip: str,
        port: int,
        on_sample: Callable[[SensorSample], None],
        buffer_size: int = 2048,
        name: str = "UDPReceiver",
    ) -> None:
        super().__init__(name=name, daemon=True)
        self._ip = ip
        self._port = port
        self._buffer_size = buffer_size
        self._on_sample = on_sample
        self._stop_event = threading.Event()
        self._socket: Optional[socket.socket] = None

    def stop(self) -> None:
        self._stop_event.set()
        if self._socket is not None:
            try:
                self._socket.close()
            except Exception:
                pass

    def run(self) -> None:
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.bind((self._ip, self._port))
        self._socket.settimeout(0.2)

        while not self._stop_event.is_set():
            try:
                data, _addr = self._socket.recvfrom(self._buffer_size)
            except socket.timeout:
                continue
            except OSError:
                break

            now_s = time.monotonic()
            try:
                line = data.decode("utf-8", errors="ignore").strip()
                if not line:
                    continue
                fields = [f for f in line.replace(";", ",").split(",") if f]
                nums = []
                for f in fields:
                    try:
                        nums.append(float(f))
                    except ValueError:
                        # ignore non-numeric tokens
                        pass
                if len(nums) < 9:
                    continue
                ax, ay, az, gx, gy, gz, mx, my, mz = nums[:9]
                sample = SensorSample(
                    acc_mps2=np.array([ax, ay, az], dtype=float),
                    gyr_rps=np.array([gx, gy, gz], dtype=float),
                    mag_uT=np.array([mx, my, mz], dtype=float),
                    timestamp_s=now_s,
                )
                self._on_sample(sample)
            except Exception:
                # Robust against malformed packets
                continue