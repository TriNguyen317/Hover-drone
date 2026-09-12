# Saved flight plots

Run offline after a mission; plotting does not access the FC.

```sh
python3 -m venv --system-site-packages .venv-plots
.venv-plots/bin/python -m pip install -r requirements-plots.txt
.venv-plots/bin/python plot_flight.py --animate
```

No CSV argument selects the newest `logs/mission_*.csv`. Or select explicitly:

```sh
.venv-plots/bin/python plot_flight.py logs/mission_20260908_170917.csv --animate
```

Outputs go to `logs/mission_<timestamp>_plots/`: position/altitude/attitude,
velocity targets versus measurements, PID, motor commands/RPM, GPS quality,
XY track, 3D trajectory PNGs and optional `trajectory_replay.gif`.
Use `--output DIRECTORY` to change destination. Existing plots there are replaced.

Inspired by test_1.m's NED trajectory and rotating X-shape UAV. Coordinates are
North/East/Down with Down = -height_agl_m. Requested X/Y are zero because the
current mission holds its calibrated anchor. No simulated wind/body rates are
invented: these are not recorded in the current CSV. GPS trajectory includes
sensor drift and is not ground truth. Missing values stay missing; no GPS means
no XY/3D plot. Aborted logs render available telemetry only.

GIF samples uniformly in log time at up to 300 frames; long flights replay faster
than real time. The displayed timestamp is the log timestamp. X-shape dimensions
are illustrative, not a measured airframe model. This is post-flight plotting,
not an automatic action in the flight-control loop.
