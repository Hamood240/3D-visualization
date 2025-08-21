import socket
import numpy as np

class UDPReceiver:
    def __init__(self, port=5006):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(0.01)
    
    def receive(self):
        try:
            data, _ = self.sock.recvfrom(5006)
            return self.parse(data.decode('utf-8'))
        except socket.timeout:
            return None
    
    def parse(self, data):
        """Parse HyperIMU data: timestamp,acc_x,acc_y,acc_z,gyr_x,gyr_y,gyr_z,mag_x,mag_y,mag_z"""
        parts = data.strip().split(',')
        if len(parts) != 10: 
            return None
            
        return {
            'accel': np.array([float(parts[1]), float(parts[2]), float(parts[3])]),
            'gyro': np.array([float(parts[4]), float(parts[5]), float(parts[6])]),
            'mag': np.array([float(parts[7]), float(parts[8]), float(parts[9])])
        }