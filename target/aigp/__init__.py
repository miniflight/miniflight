"""AI-GP simulator, controller examples, and bundled assets."""

# VQ1 build 3391: [collective, FRD roll/pitch/yaw effort], verified by motor pulses.
MOTOR_MIX = ((1,  1,  1, -1),  # front left
             (1, -1,  1,  1),  # front right
             (1,  1, -1,  1),  # rear left
             (1, -1, -1, -1))  # rear right
