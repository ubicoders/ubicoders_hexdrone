import numpy as np


def quaternion_to_euler(quaternion):
    # Flatten the quaternion in case it's a 4x1 numpy array
    w, x, y, z = quaternion
    
    # Roll (x-axis rotation)
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    
    # Pitch (y-axis rotation)
    pitch = np.arcsin(2 * (w * y - z * x))
    
    # Yaw (z-axis rotation)
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    
    # Ensure output is within [-pi, pi]
    roll = np.arctan2(np.sin(roll), np.cos(roll))
    pitch = np.arctan2(np.sin(pitch), np.cos(pitch))
    yaw = np.arctan2(np.sin(yaw), np.cos(yaw))
    
    return np.array([roll, pitch, yaw])

def euler_to_dcm_zyx(euler_angles):
    roll, pitch, yaw = euler_angles
    
    # Rotation matrix around x-axis (roll)
    R_x = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    
    # Rotation matrix around y-axis (pitch)
    R_y = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    
    # Rotation matrix around z-axis (yaw)
    R_z = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    
    # Final rotation matrix R = R_z * R_y * R_x
    R = np.dot(R_z, np.dot(R_y, R_x))
    
    return R

def quaternion_to_dcm(quaternion):
    euler_angles = quaternion_to_euler(quaternion)
    return euler_to_dcm_zyx(euler_angles)