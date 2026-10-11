"""The actual inputs and output of R1's acceleration law, before command limits."""


def r1_signals(controller):
    if controller.requested_acceleration is None:
        return None
    return dict(reference_time_s=controller.reference_time,
                position_ref_ned_m=controller.target.v.tolist(),
                velocity_ref_ned_m_s=controller.target_velocity.v.tolist(),
                acceleration_ref_ned_m_s2=controller.target_acceleration.v.tolist(),
                acceleration_request_ned_m_s2=controller.requested_acceleration.v.tolist())
