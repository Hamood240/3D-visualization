from ahrs.filters import Madgwick
import numpy as np

class PhoneTracker:
    def __init__(self, freq=100):
        self.filter = Madgwick(frequency=freq)
        self.Q = np.array([1.0, 0.0, 0.0, 0.0])  # Initial quaternion
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.dt = 1.0/freq
        
    def update(self, sensor_data):
        try:
            # Madgwick filter update
            self.Q = self.filter.updateMARG(
                self.Q,
                sensor_data['gyro'],
                sensor_data['accel'],
                sensor_data['mag']
            )
            return self.Q
        except Exception as e:
            print(f"Filter error: {e}")
            return self.Q  # Return last good value
    
    def quaternion_to_euler(self):
        w, x, y, z = self.Q
        roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        pitch = np.arcsin(2*(w*y - z*x))
        yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        return np.degrees([roll, pitch, yaw])