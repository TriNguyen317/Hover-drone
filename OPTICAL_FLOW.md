# Experimental optical-flow velocity assistance

Local implementation only; not flight validated. Existing configurations default
to `off`. Add this top-level section to config.json for monitoring:

```json
"optical_flow": {
  "mode": "monitor",
  "calibrated": false,
  "forward_axis": "y",
  "right_axis": "x",
  "forward_sign": -1,
  "right_sign": 1,
  "scale": 1.0,
  "min_quality": 70,
  "min_range_m": 0.25,
  "max_range_m": 2.5,
  "max_tilt_deg": 12.0,
  "max_speed_mps": 2.0,
  "weight": 0.7,
  "transition_s": 1.0
}
```

Mapping above is an UNVERIFIED starting convention, not mounting calibration.
Verify disarmed, props removed: translate forward/right known distances and
check computed velocity signs and magnitude; rotate without translating and
check flow/body cancellation. Range must be sensor-to-ground distance, not
ground-subtracted AGL. This approximation assumes near-level motion over a
static textured flat surface and suitably aligned/co-located rangefinder.

Only after those checks, `mode: assist` and `calibrated: true` permit blending
flow velocity into the existing GPS-position/velocity-PI cascade. GPS position
and quality gates remain unchanged. Altitude-only mode does not read/use flow.
Monitoring runs in the full-GPS mission but does not alter velocity feedback;
use read_optical_flow.py for a standalone disarmed raw monitor.

Flow is polled each control step; healthy status, quality, range, tilt and speed
are checked. Assistance ramps in. Invalid flow immediately falls back to GPS
and clears horizontal integrals; it may still produce an output step. Serial
errors propagate to existing abort handling, not silent fallback or disarm.
MSP exposes no sensor timestamp/sequence, so status and response timing cannot
prove sample freshness. Integer deg/s telemetry also limits low-speed resolution.

CSV includes raw flow/body rates, quality/status, rejection reason, range,
converted north/east velocities, blend weight, GPS velocities and selected
source. Existing velocity/PID columns describe the blended control feedback.

IMPORTANT: `assist` is not GPS drift removal or optical-flow position hold.
A drifting GPS target still drives the outer loop. A separate relative-position
estimator with flow integration, bounded slow GPS correction and explicit
sensor-loss handling is needed for that purpose. Do not treat these unit tests
as aircraft qualification. No automatic deployment or flight was performed.

## Relative mode: local experimental implementation

Use `mode: relative` only after verifying physical axis signs, scale and rotation
compensation and setting `calibrated: true`. Defaults remain off. This is full-GPS
relative hover, not a new altitude-only mode or global waypoint navigation.

INAV performs sensor estimation internally. Pi reads attitude, N/E/up position
and N/E/up velocity together with MSP command 0x2220 once per control step.
Altitude PID uses INAV local up relative to a post-ARM, throttle-low origin so an
INAV arm-time estimator reset cannot create a false altitude step. The X/Y
origin is captured at the first position-enabled step after liftoff, NOT at
ground calibration; subsequent position is the direct local-pose displacement,
not a Pi-side integral. Inner velocity PI uses local-pose velocity. Raw flow is
only a quality watchdog and log source.

Pi performs no second GPS correction or sensor fusion in relative mode. GPS may
already influence INAV local pose according to the INAV estimator configuration;
therefore its GPS/flow estimator weights must be inspected before flight.

GPS is optional for Pi-side relative-mode admission and monitoring. A failed
GPS gate does not stop relative hold, and raw GPS distance no longer causes the
8 m abort; the independent relative-position envelope remains. Other modes are
unchanged.

Invalid flow gets a short watchdog grace period (`flow_loss_timeout_s`, default
0.4 s); continuous loss then aborts through existing operator-takeover handling,
without automatic airborne disarm, GPS fallback or a new emergency landing
procedure. On final descent ONLY, below min_range_m, X/Y commands are centered
and normal vertical landing
continues; horizontal drift is possible in this final segment.

New log fields: position_source, relative_north_m, relative_east_m,
gps_raw_north_m, gps_raw_east_m, local pose position/velocity and flow watchdog.
north_m/east_m/position_error_m now describe the selected control position.
Legacy gps_* PID error columns mean relative errors when FLOW_RELATIVE is active.
Before activation, estimator fields are blank. Plotted tracks are estimates,
not ground truth. No claim is made about real-world improvement from unit tests.

Use the read-only calibration assistant only with every propeller removed:

```bash
python3 calibrate_optical_flow.py --config config.json --props-removed
```

It records one known forward and one known right translation, writes a new
recommendation JSON without overwriting an old result, never changes config.json,
and deliberately leaves `calibrated=false`. Repeat it and verify consistency.

The simulation uses an idealized sensor and does not validate mounting, gyro
lag/cancellation, stale data, illumination or ground effects. Physical checks
with props removed and verified takeover are still required before flight.
