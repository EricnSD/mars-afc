# fusion.py - Sensor fusion utilities for rocket flight data processing
import math

def calculate_rt_alpha(dt, cutoff_freq=15.0):
    """Computes a dynamic low-pass filter coefficient based on loop dt."""
    time_constant = 1.0 / (2.0 * math.pi * cutoff_freq)
    return time_constant / (time_constant + dt)

def low_pass_filter(current_raw, previous_filtered, alpha):
    """Applies a single-pole low-pass filter to clean up raw gyro noise."""
    return (alpha * previous_filtered) + ((1.0 - alpha) * current_raw)

def unspin_body_rates_to_global(gy_body, gz_body, fused_roll):
    cos_r = math.cos(fused_roll)
    sin_r = math.sin(fused_roll)
    
    # Correct inverse transformation to unspin the body frame back to global horizons
    gx_global = gy_body * cos_r - gz_body * sin_r
    gy_global = gy_body * sin_r + gz_body * cos_r
    
    return gx_global, gy_global

def resolve_total_vertical_tilt(pitch_rad, yaw_rad):
    """Calculates total angular spatial tilt (Theta) away from vertical gravity."""
    return math.sqrt(pitch_rad**2 + yaw_rad**2)

def rotate_body_accel_to_global(ax, ay, az, pitch_rad, yaw_rad):
    """
    Rotates body-frame accelerations into a stable global Earth frame
    assuming X is the longitudinal/vertical rocket axis.
    
    Sequence: Pitch around Y, followed by Yaw around Z.
    """
    cos_p, sin_p = math.cos(pitch_rad), math.sin(pitch_rad)
    cos_y, sin_y = math.cos(yaw_rad), math.sin(yaw_rad)
    
    # 3D Matrix multiplication explicitly mapped for X-longitudinal body frame
    ax_global = (sin_p * sin_y * ax) + (cos_y * ay) - (cos_p * sin_y * az)
    ay_global = (-sin_p * cos_y * ax) + (sin_y * ay) + (cos_p * cos_y * az)
    az_global = (cos_p * ax) + (sin_p * az)
    
    # Subtract Earth's gravity (9.81 m/s^2) ONLY from the global vertical axis (Z)
    az_global -= 9.81
    
    return ax_global, ay_global, az_global

def update_gyro_bias_complementary(fused_roll, gyro_bias_x, my, mz, dt, k_prop=2.0, k_bias=0.03):
    roll_angle_mag = math.atan2(my, mz)
    
    roll_error = roll_angle_mag - fused_roll
    roll_error = (roll_error + math.pi) % (2.0 * math.pi) - math.pi
        
    # Proportional Step: Direct, immediate dampening adjustment to the live angle
    fused_roll_corrected = fused_roll + (k_prop * roll_error * dt)
    
    # Integral Step: Slow, long-term trimming of the gyro bias scaled by dt
    gyro_bias_x_corrected = gyro_bias_x - (k_bias * roll_error * dt)
    
    return fused_roll_corrected, gyro_bias_x_corrected
