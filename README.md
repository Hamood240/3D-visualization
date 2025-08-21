# Real-Time Motion Tracking & Visualization

This project receives IMU data (accelerometer m/s^2, gyroscope rad/s, magnetometer µT) from HyperIMU over UDP, performs sensor fusion (Madgwick), estimates speed and distance via integration with ZUPT and filtering, and visualizes the phone orientation and motion in 3D using VPython.

## Features
- Real-time orientation via Madgwick (AHRS)
- Speed and distance estimation with gravity removal, high/low-pass filtering, ZUPT
- Rotation tracking (yaw/pitch/roll) with drift control
- Multithreaded: UDP receive, processing, visualization
- VPython visualizer with live metrics overlay
- Headless demo mode for environments without GUI

## Install
```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
```

## Run (UDP from HyperIMU)
```bash
python -m rt_motion.main --udp-port 5555 --rate 100
```

## Run (demo, headless)
```bash
python -m rt_motion.main --demo --headless --rate 100
```

## HyperIMU
Configure HyperIMU to send CSV over UDP with accelerometer (m/s^2), gyroscope (rad/s), magnetometer (µT). The parser attempts to auto-detect column order; you can override via CLI.

## Tuning flags
- --acc-highpass-alpha, --acc-lowpass-alpha
- --zupt-acc-thresh, --zupt-gyro-thresh
- --kalman-use (enable simple 1D Kalman on velocity)

## Notes
- Distance from double integration is sensitive to bias. Keep the phone stationary for a second at start for initial bias settle.
- Headless mode prints periodic stats and skips VPython.