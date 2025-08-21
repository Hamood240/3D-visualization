from __future__ import annotations
import argparse
import signal
import threading
import time
from queue import Queue
from typing import Optional
import numpy as np

from .config import DEFAULTS
from .models import SensorSample
from .udp_receiver import UDPReceiver
from .processor import MotionProcessor
from .visualizer import Visualizer


class DemoGenerator(threading.Thread):
    """Generate synthetic IMU data at a fixed rate for smoke testing."""

    def __init__(self, out_queue: Queue, rate_hz: float) -> None:
        super().__init__(name="DemoGenerator", daemon=True)
        self._q = out_queue
        self._dt = 1.0 / float(rate_hz)
        self._stop = threading.Event()
        self._t = 0.0

    def stop(self) -> None:
        self._stop.set()

    def run(self) -> None:
        last = time.monotonic()
        while not self._stop.is_set():
            now = time.monotonic()
            dt = now - last
            last = now
            self._t += dt

            # Simple scenario: rotate yaw slowly, move forward then stop
            yaw_rate = np.radians(20.0)  # rad/s
            gyr = np.array([0.0, 0.0, yaw_rate])

            # Acceleration profile: 2s accelerate at 1 m/s^2, 2s coast (0), 2s brake -1
            phase = int((self._t // 2.0) % 3)
            acc_body = np.array([1.0, 0.0, 0.0]) if phase == 0 else (np.array([0.0, 0.0, 0.0]) if phase == 1 else np.array([-1.0, 0.0, 0.0]))
            # Add gravity in body frame approximately as +Z; we keep it simple for demo
            acc_body = acc_body + np.array([0.0, 0.0, 9.80665])

            mag = np.array([30.0, 0.0, -40.0])

            sample = SensorSample(acc_mps2=acc_body, gyr_rps=gyr, mag_uT=mag, timestamp_s=now)
            self._q.put(sample)
            time.sleep(self._dt)


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Real-Time IMU Motion Tracking")
    p.add_argument("--udp-ip", default=DEFAULTS.udp_ip)
    p.add_argument("--udp-port", type=int, default=DEFAULTS.udp_port)
    p.add_argument("--rate", type=float, default=DEFAULTS.sample_rate_hz, help="Sample rate Hz")
    p.add_argument("--target-fps", type=int, default=DEFAULTS.target_fps)
    p.add_argument("--headless", action="store_true")
    p.add_argument("--demo", action="store_true", help="Run demo generator instead of UDP")

    p.add_argument("--acc-lowpass-alpha", type=float, default=DEFAULTS.acc_lowpass_alpha)
    p.add_argument("--acc-highpass-alpha", type=float, default=DEFAULTS.acc_highpass_alpha)
    p.add_argument("--zupt-acc-thresh", type=float, default=DEFAULTS.zupt_acc_thresh)
    p.add_argument("--zupt-gyro-thresh", type=float, default=DEFAULTS.zupt_gyro_thresh)
    p.add_argument("--zupt-min-duration", type=float, default=DEFAULTS.zupt_min_duration_s)
    p.add_argument("--kalman-use", action="store_true", default=DEFAULTS.kalman_use)
    p.add_argument("--kalman-process-var", type=float, default=DEFAULTS.kalman_process_var)
    p.add_argument("--kalman-measure-var", type=float, default=DEFAULTS.kalman_measure_var)
    return p


def main() -> None:
    args = build_arg_parser().parse_args()

    raw_queue: Queue = Queue(maxsize=1000)
    frame_queue: Queue = Queue(maxsize=100)

    processor = MotionProcessor(
        sample_rate_hz=args.rate,
        acc_lowpass_alpha=args.acc_lowpass_alpha,
        acc_highpass_alpha=args.acc_highpass_alpha,
        zupt_acc_thresh=args.zupt_acc_thresh,
        zupt_gyro_thresh=args.zupt_gyro_thresh,
        zupt_min_duration_s=args.zupt_min_duration,
        kalman_use=args.kalman_use,
        kalman_process_var=args.kalman_process_var,
        kalman_measure_var=args.kalman_measure_var,
    )

    visualizer = Visualizer(frame_queue=frame_queue, target_fps=args.target_fps, headless=args.headless)

    # Producer: UDP or Demo
    producer: Optional[threading.Thread]
    if args.demo:
        producer = DemoGenerator(raw_queue, rate_hz=args.rate)
    else:
        def on_sample(sample: SensorSample) -> None:
            raw_queue.put(sample)
        producer = UDPReceiver(args.udp_ip, args.udp_port, on_sample)

    # Processor thread
    stop_event = threading.Event()

    def processing_loop() -> None:
        last_ts: Optional[float] = None
        while not stop_event.is_set():
            try:
                sample: SensorSample = raw_queue.get(timeout=0.5)
            except Exception:
                continue
            if last_ts is not None and sample.timestamp_s <= last_ts:
                # enforce monotonic timestamps
                sample = SensorSample(
                    acc_mps2=sample.acc_mps2,
                    gyr_rps=sample.gyr_rps,
                    mag_uT=sample.mag_uT,
                    timestamp_s=last_ts + 1e-3,
                )
            frame = processor.process(sample)
            last_ts = sample.timestamp_s
            # Drop frames if visualizer lags
            if frame_queue.full():
                try:
                    frame_queue.get_nowait()
                except Exception:
                    pass
            frame_queue.put(frame)

    processing_thread = threading.Thread(target=processing_loop, name="Processor", daemon=True)

    def shutdown(signum=None, frame=None):
        stop_event.set()
        visualizer.stop()
        if hasattr(producer, "stop"):
            try:
                producer.stop()  # type: ignore[attr-defined]
            except Exception:
                pass

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    # Start
    producer.start()
    processing_thread.start()
    visualizer.start()

    # Join visualizer thread (others are daemon)
    visualizer.join()


if __name__ == "__main__":
    main()