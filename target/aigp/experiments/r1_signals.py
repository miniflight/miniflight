"""The actual inputs and output of R1's acceleration law, before command limits."""

from miniflight import vehicle

def r1_signals(controller):
    if controller.requested_acceleration is None:
        return None
    return dict(reference_time_s=controller.reference_time,
                position_ref_ned_m=controller.target.v.tolist(),
                velocity_ref_ned_m_s=controller.target_velocity.v.tolist(),
                acceleration_ref_ned_m_s2=controller.target_acceleration.v.tolist(),
                acceleration_request_ned_m_s2=list(controller.requested_acceleration),
                attitude_used_rad=list(controller.angles),
                heading_anchor_rad=controller.target_yaw,
                tilt_time_s=controller.attitude_estimate.time,
                tilt_rotation_wxyz=controller.attitude_estimate.rotation.q.tolist())


def motor_signals(controller):
    return dict(imu_time_s=vehicle.state.time,
                rate_time_s=controller.time,
                integral_dt_s=controller.integral_dt,
                rate_ref_frd_rad_s=list(controller.rate_request),
                gyro_frd_rad_s=list(controller.gyro),
                torque_request_frd=list(controller.torque),
                integral_effort_frd=list(controller.integral),
                motor_request=list(controller.motor_request))
