# Stock VQ2 checkpoint

Lap 1671 finished in **19.927881240 s** with all 17 gates and zero contacts. Lap 1658 finished in **19.929412841 s** on the same carried map and fast trajectory. The ten-lap repeat produced one clean sub-20 finish, and the longest streak was one.

The pilot used camera frames, IMU and its own commands. The carried map came from earlier flights. The native race clock and gate credits supplied the score. The video uses the 110 camera frames saved during lap 1671 at their recorded timing, with frame holds between captures.

`controller.tar.gz` contains the runtime source, input maps, line, tracking calibration and frozen plan. It also includes the inputs and results for the unified attitude tests. The unified tracker finished a clean **23.082395553 s** lap at fallback pace. Both arms finished zero of four on the matched fast line.

Extract the archive into an empty directory. With the Python dependencies in `checkpoint.json`, run `python run.py` to check the source hashes and plan binding. To fly, open stock VQ2 Practice and run `python run.py --run /path/to/new/output`. The default is one attempt.
