Connect the flight controller and run this from the repository root.

```sh
python3.11 -m config.serve
```

Open [127.0.0.1:8002](http://127.0.0.1:8002/). Use `--device` to select a port
if more than one USB modem is present. **Save Blackbox** downloads the flash
contents. Live readings resume when the read finishes.
