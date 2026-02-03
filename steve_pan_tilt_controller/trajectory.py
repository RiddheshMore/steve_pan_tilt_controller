"""Trajectory generation utilities for pan-tilt control."""
import math
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

def create_single_goal_trajectory(current_pan_rad, current_tilt_rad, target_pan_rad, target_tilt_rad, speed_rad_s):
    """
    Creates a single point trajectory from current to target.
    Returns: (points_list, duration_in_seconds)
    """
    point = JointTrajectoryPoint()
    # Mapping: joint_names=['pan_tilt_pan_motor_joint', 'pan_tilt_tilt_motor_joint']
    # Index 0 = pan, Index 1 = tilt
    point.positions = [target_pan_rad, target_tilt_rad]
    point.velocities = [0.0, 0.0]
    
    # Duration logic based on ACTUAL distance
    dist_pan = abs(target_pan_rad - current_pan_rad)
    dist_tilt = abs(target_tilt_rad - current_tilt_rad)
    dist = max(dist_pan, dist_tilt)
    
    if speed_rad_s <= 1e-6:
         duration = 2.0
    else:
         duration = dist / speed_rad_s
    duration = max(duration, 2.0) # Minimum 2.0s to ensure simulation stability

    sec = int(duration)
    nanosec = int((duration - sec) * 1e9)
    point.time_from_start = Duration(sec=sec, nanosec=nanosec)
    
    return [point], duration

def create_elliptical_trajectory(center_pan_rad, pan_amp_rad, center_tilt_rad, tilt_amp_rad, speed_rad_s):
    """
    Creates an elliptical trajectory centered at (center_pan, center_tilt).
    Returns: (points_list, period_in_seconds)
    """
    A = pan_amp_rad
    B = tilt_amp_rad
    
    max_amplitude = max(abs(A), abs(B))
    if max_amplitude < 1e-6:
        omega = 1.0 
    else:
        omega = speed_rad_s / max_amplitude
        
    period = 2 * math.pi / omega
    
    # Generate points
    # Resolution: 10 points per second or minimum 20 points
    num_points = max(int(period * 10), 20) 
    time_step = period / num_points
    
    points = []
    for i in range(1, num_points + 1): # Start from 1 to avoid time=0 in first point
        t = i * time_step
        # Parametric equations
        # Pan = Center + A * cos(omega * t)
        # Tilt = Center + B * sin(omega * t)
        
        p_val = center_pan_rad + A * math.cos(omega * t)
        t_val = center_tilt_rad + B * math.sin(omega * t)
        
        # Velocities (derivatives)
        v_p = -A * omega * math.sin(omega * t)
        v_t = B * omega * math.cos(omega * t)
        
        point = JointTrajectoryPoint()
        # Mapping: joint_names=['pan_tilt_pan_motor_joint', 'pan_tilt_tilt_motor_joint']
        # Index 0 = pan, Index 1 = tilt
        point.positions = [p_val, t_val]
        # point.velocities = [v_p, v_t] # Let controller calculate velocities
        
        # Time from start
        t_sec = int(t)
        t_nano = int((t - t_sec) * 1e9)
        point.time_from_start = Duration(sec=t_sec, nanosec=t_nano)
        
        points.append(point)
        
    return points, period
