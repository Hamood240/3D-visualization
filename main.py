from udp_reciver import UDPReceiver
from sensor_fussion import PhoneTracker
from visualisation import PhoneVisualizer
import time
import queue
from vpython import rate, vector  # Do NOT import 'quaternion'

# Initialize components
receiver = UDPReceiver(port=5006)  # Match HyperIMU port
tracker = PhoneTracker(freq=100)
visualizer = PhoneVisualizer()

# Create a thread-safe queue for communication
data_queue = queue.Queue()

def data_processing_thread():
    """Thread for handling data processing"""
    while True:
        data = receiver.receive()
        if data:
            # Apply low-pass filter to raw data (optional)
            # data['accel'] = low_pass(data['accel'])
            
            # Update tracker
            quat = tracker.update(data)
            data_queue.put(quat)  # Send to main thread
        time.sleep(0.005)  # Adjust based on your update rate

# Start processing thread
import threading
thread = threading.Thread(target=data_processing_thread, daemon=True)
thread.start()

# Main visualization loop
while True:
    rate(100)  # VPython rate controller (100 Hz)
    
    # Process any new data
    while not data_queue.empty():
        quat = data_queue.get()
        visualizer.update(quat)

# Example usage for testing quaternion visualization
from time import sleep

vis = PhoneVisualizer()

# Simulated quaternion stream (replace with your real data stream)
while True:
    q = [0.92388, 0.0, 0.38268, 0.0]  # Example: 45° rotation around Y-axis
    vis.update(q)
    rate(30)  # 30 updates per second