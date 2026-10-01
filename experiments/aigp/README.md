# AIGP experiments

`probe_vq1_motor.py` is a historical raw-MAVLink experiment. Its actuator mapping
has not been validated. It bypasses the shared runner, arms directly, and restores
idle outputs at the end rather than disarming. It is retained as research material,
not as a supported flight example or a miniflight control plane.

Use the examples under `examples/aigp/` for the supported runner lifecycle.

`vq1_arm_probe.py` is another historical diagnostic. It uses the removed
`SimulatorClient._message` method and is retained for reference only.
