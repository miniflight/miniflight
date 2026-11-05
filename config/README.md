Connect the flight controller and run this from the repository root.

```sh
python3.11 -m config.serve
```

Open [127.0.0.1:8002](http://127.0.0.1:8002/). Use `--device` to select a port
if more than one USB modem is present. **Save Blackbox** downloads the flash
contents. Live readings resume when the read finishes.

For gyro replay, install the existing monitor dependencies and pass a decoded CSV.
Use `blackbox_decode --unit-frame-time us --unit-rotation deg/s` for the export.

```sh
python3.11 -m pip install -e ".[monitor]"
python3.11 -m config.serve --replay path/to/recording.csv
```

**Play recording** reconstructs rotation from the first frame, using every gyro
sample and the recorded intervals. **Restart** begins at that frame again.
Replay currently requires consecutive loop indices.
