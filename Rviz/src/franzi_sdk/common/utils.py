import time
import numpy as np
import sys
import os
import scipy.spatial.transform as st
from scipy.spatial.transform import Rotation as R

# Windows 定时器精度提升到 1ms，避免 time.sleep() 默认 15.6ms 分辨率导致 precise_wait 超时
if sys.platform == 'win32':
    try:
        import ctypes
        ctypes.windll.winmm.timeBeginPeriod(1)
    except Exception:
        pass

RED = "\033[31m"
GREEN = "\033[32m"
YELLOW = "\033[33m"
BLUE = "\033[34m"
RESET = "\033[0m"

def add_pose_increment(base_pose: np.ndarray, 
                       delta_pose: np.ndarray, 
                       degrees: bool = True, 
                       rotation_order: str = 'zyx',
                       translation_scale: float = 1.0) -> np.ndarray:

    if not isinstance(base_pose, np.ndarray) or not isinstance(delta_pose, np.ndarray):
        raise TypeError(f"base_pose和delta_pose必须是numpy数组！当前base_pose类型={type(base_pose)}, delta_pose类型={type(delta_pose)}")
    if base_pose.shape != (6,) or delta_pose.shape != (6,):
        raise ValueError(f"base_pose和delta_pose必须是6维数组！当前base_pose.shape={base_pose.shape}, delta_pose.shape={delta_pose.shape}")
    if not np.isfinite(base_pose).all() or not np.isfinite(delta_pose).all():
        raise ValueError("base_pose或delta_pose包含NaN/Inf，无法计算！")

    ref_xyz = base_pose[:3]     
    delta_xyz = delta_pose[:3]   
    ref_rpy = base_pose[3:]       
    delta_rpy = delta_pose[3:]    
    final_xyz = ref_xyz + delta_xyz * translation_scale
    rot_ref = R.from_euler(rotation_order, ref_rpy, degrees=degrees)
    rot_delta = R.from_euler(rotation_order, delta_rpy, degrees=degrees)
    rot_final = rot_delta * rot_ref
    final_rpy = rot_final.as_euler(rotation_order, degrees=degrees)
    final_pose = np.concatenate([final_xyz, final_rpy])
    return final_pose

def convert_to_rotation(increment_data):
    if isinstance(increment_data, st.Rotation):
        return increment_data
    elif isinstance(increment_data, (np.ndarray, list)):
        euler_angles = increment_data[3:] if len(increment_data) >= 3 else [0, 0, 0]
        return st.Rotation.from_euler('zyx', euler_angles, degrees=True)
    else:
        return st.Rotation.identity()

def v6d2tm(vec6d):
    T = np.eye(4)
    T[:3, 3] = vec6d[:3]
    roll = vec6d[3]
    pitch = vec6d[4]
    yaw = vec6d[5]
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(roll), -np.sin(roll)],
                   [0, np.sin(roll), np.cos(roll)]])
    Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                   [0, 1, 0],
                   [-np.sin(pitch), 0, np.cos(pitch)]])
    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                   [np.sin(yaw), np.cos(yaw), 0],
                   [0, 0, 1]])
    rot_matrix = Rx @ Ry @ Rz
    T[:3, :3] = rot_matrix
    return T

def tm2v6d(T):
    vec6d = np.zeros(6)
    vec6d[:3] = T[:3, 3]
    R_mat = T[:3, :3]
    
    if abs(R_mat[0, 2]) < 1 - 1e-6:
        pitch = np.arcsin(R_mat[0, 2])
        roll = np.arctan2(-R_mat[1, 2], R_mat[2, 2])
        yaw = np.arctan2(-R_mat[0, 1], R_mat[0, 0])
    else:
        yaw = 0.0
        if R_mat[0, 2] > 0:
            pitch = np.pi/2
            roll = yaw + np.arctan2(R_mat[1, 0], R_mat[2, 0])
        else:
            pitch = -np.pi/2
            roll = -yaw + np.arctan2(-R_mat[1, 0], -R_mat[2, 0])
    
    vec6d[3] = roll
    vec6d[4] = pitch
    vec6d[5] = yaw
    return vec6d

def rot3x3_to_tf4x4(rot_matrix):
    tf_matrix = np.eye(4)
    tf_matrix[:3, :3] = rot_matrix
    return tf_matrix

def precise_wait(t_end: float, slack_time: float = 0.001):
    """精确等待到指定时间"""
    t_start = time.monotonic()
    t_wait = t_end - t_start
    if t_wait > 0:
        t_sleep = t_wait - slack_time
        if t_sleep > 0:
            time.sleep(t_sleep)
        while time.monotonic() < t_end:
            pass
    return