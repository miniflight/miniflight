# MonoRace on stock VQ2

This checkpoint flew **18.181 s, 17/17, zero contacts** with a monocular camera, IMU, a carried map and four direct motor commands. The earlier five-start batch with the same actor and map ran **3/5** clean sub-20 laps. In the reference batch, start 1 hit an obstacle and start 3 was stopped while waiting for a fresh countdown.

Extract `controller.tar.gz` into an empty directory. From there, run `python3.11 -m pip install -r requirements.txt` in a virtual environment, then `python3.11 run.py` to verify the files and replay the recorded actor decisions.

Open **AI-GP VQ2 Track Practice** in the official simulator, then run `python3.11 run.py --run ./new-run --starts 5`. Pitwall uses port 8766. If the simulator stays on the result screen, click **Retry**. The archive includes the wire audit in `verification.json` and the [TU Delft MonoRace release](https://github.com/tudelft/optimal_quad_control_RL/tree/a2rl2025) license.
