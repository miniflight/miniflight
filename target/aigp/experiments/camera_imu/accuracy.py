"""Post-flight errors and observation ages; native motion stays in this scorer."""

import csv

import numpy as np


def statistics(values):
    values = np.asarray(values, dtype=float)
    if not len(values):
        return dict(count=0)
    return dict(count=len(values), median=float(np.median(values)), p95=float(np.quantile(values, .95)),
                maximum=float(np.max(values)), rms=float(np.sqrt(np.mean(values**2))))


def score_estimates(root, motion, ticks, center, rotation, observations):
    truth = lambda stamp, start: np.array([np.interp(stamp, motion[:, 0], col) for col in motion[:, start:start+3].T])
    errors = {key: [] for key in ("camera_position_m", "camera_velocity_m_s", "fused_position_m", "fused_velocity_m_s",
                                  "unfused_position_m", "unfused_velocity_m_s", "capture_to_output_ms", "velocity_fit_age_ms",
                                  "accepted_camera_position_m")}
    rows = []
    for tick in ticks:
        if "unfused_position" not in tick:
            continue
        stamp = tick["imu_time"]
        position = truth(stamp, 1)-center
        velocity = truth(stamp, 4)
        for prefix in ("fused", "unfused"):
            p = tick["position" if prefix == "fused" else "unfused_position"]
            v = tick["velocity" if prefix == "fused" else "unfused_velocity"]
            errors[prefix+"_position_m"].append(np.linalg.norm(rotation @ (np.array(p)-tick["gate_center"])-position))
            errors[prefix+"_velocity_m_s"].append(np.linalg.norm(rotation @ v-velocity))
    for observed in observations:
        stamp = observed["capture_time"]
        position_error = rotation @ observed["position_relative_gate"]-(truth(stamp, 1)-center)
        errors["camera_position_m"].append(np.linalg.norm(position_error))
        if observed.get("correction_enabled"):
            errors["accepted_camera_position_m"].append(np.linalg.norm(position_error))
        row = dict(image_id=observed["image_id"], capture_time=stamp,
                   position_error_m=float(np.linalg.norm(position_error)),
                   capture_to_output_ms=observed["capture_to_output_s"]*1000,
                   true_range_m=float(np.linalg.norm(truth(stamp, 1)-center)),
                   measured_range_m=float(np.linalg.norm(observed["position_relative_gate"])),
                   reprojection_rmse_px=observed.get("reprojection_rmse_px"))
        row.update({"position_error_"+axis+"_m":float(value) for axis, value in zip("ned", position_error)})
        errors["capture_to_output_ms"].append(row["capture_to_output_ms"])
        fitted = observed["velocity"]
        if fitted is not None:
            error = rotation @ fitted["velocity"]-truth(fitted["time"], 4)
            row.update(velocity_time=fitted["time"], velocity_error_m_s=float(np.linalg.norm(error)),
                       velocity_fit_age_ms=(stamp-fitted["time"])*1000,
                       window_s=fitted["window_s"], observations=fitted["observations"])
            row.update({"velocity_error_"+axis+"_m_s":float(value) for axis, value in zip("ned", error)})
            errors["camera_velocity_m_s"].append(row["velocity_error_m_s"])
            errors["velocity_fit_age_ms"].append(row["velocity_fit_age_ms"])
        rows.append(row)
    if rows:
        with (root/"camera-errors.csv").open("w") as file:
            writer = csv.DictWriter(file, fieldnames=("image_id", "capture_time", "position_error_m", "capture_to_output_ms",
                "true_range_m", "measured_range_m", "reprojection_rmse_px", "position_error_n_m", "position_error_e_m", "position_error_d_m",
                "velocity_time", "velocity_error_m_s", "velocity_fit_age_ms", "window_s", "observations",
                "velocity_error_n_m_s", "velocity_error_e_m_s", "velocity_error_d_m_s"))
            writer.writeheader()
            writer.writerows(rows)
    result = {key: statistics(values) for key, values in errors.items()}
    result["camera_position_updates"] = int(sum(np.linalg.norm(row["position_correction"]) > 0 for row in observations))
    result["camera_velocity_updates"] = int(sum(np.linalg.norm(row["velocity_correction"]) > 0 for row in observations))
    return result
