from vpython import *
import numpy as np

class PhoneVisualizer:
    def __init__(self):
        self.scene = canvas(title='Phone Orientation', width=1200, height=800)
        self.scene.range = 8
        self.scene.forward = vector(0, -1, -1)  # Better viewing angle

        # Create phone model
        self.phone = box(
            size=vector(6, 12, 0.8), 
            color=color.blue,
            opacity=0.8
        )

        # Add screen to phone
        self.screen = box(
            size=vector(5.5, 11, 0.1),
            color=color.white,
            pos=vector(0, 0, 0.35)
        )

        # Add orientation indicator
        self.top_arrow = arrow(
            pos=vector(0, 0, 0),
            axis=vector(0, 6, 0),
            color=color.red,
            shaftwidth=0.3
        )

        # Add orientation label
        self.orientation_label = label(
            pos=vector(-10, 10, 0),
            text="Orientation: Waiting for data...",
            height=20,
            box=False
        )

    def update(self, quat):
        """Update phone orientation based on quaternion"""
        rot_matrix = self.quaternion_to_rotation_matrix(quat)

        # Extract axis and up vectors
        new_axis = vector(*rot_matrix[:, 1])  # Y-axis direction
        new_up = vector(*rot_matrix[:, 2])    # Z-axis direction

        # Update orientation of phone components
        self.phone.axis = new_axis
        self.phone.up = new_up
        self.screen.axis = new_axis
        self.screen.up = new_up
        self.top_arrow.axis = 6 * new_axis
        self.top_arrow.up = new_up

        # Update orientation info
        roll, pitch, yaw = self.quaternion_to_euler(quat)
        self.orientation_label.text = (
            f"Roll: {roll:.1f}°\nPitch: {pitch:.1f}°\nYaw: {yaw:.1f}°"
        )

    @staticmethod
    def quaternion_to_rotation_matrix(q):
        """Convert quaternion to 3x3 rotation matrix"""
        w, x, y, z = q
        return np.array([
            [1 - 2*y*y - 2*z*z,   2*x*y - 2*z*w,     2*x*z + 2*y*w],
            [2*x*y + 2*z*w,       1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
            [2*x*z - 2*y*w,       2*y*z + 2*x*w,     1 - 2*x*x - 2*y*y]
        ])

    @staticmethod
    def quaternion_to_euler(q):
        """Convert quaternion to Euler angles (degrees)"""
        w, x, y, z = q
        roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        pitch = np.arcsin(2*(w*y - z*x))
        yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        return np.degrees([roll, pitch, yaw])
