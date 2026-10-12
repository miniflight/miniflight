"""Convert recorded AIGP inputs and sent commands into ordinary SI quantities."""

import math


def quantities(signals, position, velocity, motors, pose_age, imu_age, acceleration_limit=None):
    target_position = signals["position_ref_ned_m"]
    target_velocity = signals["velocity_ref_ned_m_s"]
    north_error, east_error, down_error = (wanted-actual for wanted, actual in zip(target_position, position, strict=True))
    acceleration = signals["acceleration_request_ned_m_s2"]
    return dict(time_s=signals["reference_time_s"], speed_ref_m_s=math.hypot(*target_velocity[:2]),
                speed_m_s=math.hypot(*velocity[:2]), north_error_m=north_error, east_error_m=east_error, down_error_m=down_error,
                pitch_ref_rad_s=signals["rate_ref_frd_rad_s"][1], pitch_rad_s=signals["gyro_frd_rad_s"][1],
                motor_min=min(motors), motor_mean=sum(motors)/len(motors), motor_max=max(motors),
                acceleration_request_m_s2=math.hypot(*acceleration[:2]), acceleration_limit_m_s2=acceleration_limit,
                pose_age_s=pose_age, imu_age_s=imu_age)
