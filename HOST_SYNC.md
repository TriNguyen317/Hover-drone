# Host synchronization record

## Pending LOCAL ONLY: 2026-09-12 optical flow relative hold

User requests no host connection now. No deployment or flight performed.
When explicitly asked to sync, back up host files and preserve config.json;
copy flight_controller/config.py, inav.py, mission.py, models.py, optical_flow.py,
simulator.py, calibrate_optical_flow.py, plot_flight.py, config.example.json,
tests/test_optical_flow.py, OPTICAL_FLOW.md and this record.
Run all controller tests on host. Do NOT automatically enable flight settings.
New mode `relative` uses INAV full local pose directly for attitude/Z/X/Y and
velocity; Pi no longer integrates raw flow or applies GPS correction. It is gated
by verified optical-flow setup and calibrated=true; default remains off. See
OPTICAL_FLOW.md for limits and failure handling.

## Latest deployment: 192.168.50.148

Deployed cascaded X/Y control and removal of typed confirmation to
raspi4@192.168.50.148:/home/raspi4/Documents/drone/new_code.
21 unit tests passed on the Pi. Host config.json preserved; new control
parameters use defaults when omitted. Backup: .codex_backups/before_cascade_.tgz.
The pending sections below describe changes now included in this deployment.

## Pending: cascaded horizontal controller

Deploy flight_controller/{mission,config,models,inav,simulator}.py,
tests/test_controller.py, README.md and config.example.json together.
Position P now produces a vector-limited velocity target; velocity PI uses
GNSS speed/course and produces tilt. Legacy position PID config is unused.
New defaults: position_velocity_kp=0.5, horizontal_speed_limit_mps=0.5,
velocity_tilt_kp=5.0, velocity_tilt_ki=0.3. These are simulation gains.
Preserve the host config.json; omitted new fields use these defaults.

## Pending: remove typed confirmation

Local `flight_controller/mission.py` now announces the selected mode and moves
directly to physical CH8 takeover without calling input. `--fly`, preflight,
override verification and ARM checks remain required. README and tests updated.
Host connection timed out; this change has not been deployed.

Target: `/home/raspi4/Documents/drone/new_code` on `raspberry-drone2`.

## Deployed 2026-09-08

- `flight_controller/mission.py`: CSV now records North/East GPS PID error,
  P/I/D, raw output, vector-limited tilt and yaw-rotated body tilt. It also
  records the calibrated GPS reference and the GPS measurement in every row.
- `tests/test_controller.py`: verifies GPS PID fields and blank values in
  altitude-only mode.
- `README.md`: documents the new CSV fields and corrects the altitude-hover
  GPS PID description.

Run the controller test suite after copying these files to the host.

Status: deployed to the target above and verified with 20 passing unit tests.
The previous host files are preserved in
`.codex_backups/pid_gps_20260908/` on the host.

## Deployed after that update

- Save every ground-calibration sample and the computed reference/statistics
  to `logs/calibration_YYYYMMDD_HHMMSS.csv`.

Status: deployed and verified on the host with 20 passing unit tests. GPS and
ground height continue to use median; this deployment does not change the
calibration aggregation algorithm.
