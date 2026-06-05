# Object tracking with siyi_ros2

This document describes the high-rate command interface added to
`siyi_ros2` for closed-loop object tracking with the SIYI A8 mini.

The use case it targets: a Jetson Orin NX connected over Ethernet to
an A8 mini, with a detector + tracking node in a separate ROS2 package
that commands the gimbal to keep a fast-moving target centered in the
image while the airframe is also moving fast.

## Architecture

```
[detector node] ──bbox──▶ [tracking node]
                                │
                                ├── pixel error → PID → yaw/pitch rate (deg/s)
                                │
                                ▼
                       /siyi/cmd/rate  (30–100 Hz)
                                │
[FCU bridge] ──body rate──▶ [siyi_node] ──rotate_nowait──▶ A8 mini
                                ▲                              │
                                └────/siyi/attitude (50 Hz)────┘
                                          │
                          /siyi/saturation (when clamped)
```

The tracker only talks to `siyi_ros2` over **topics**, never services.
Services in this package are reserved for non-real-time setup
(centering, mode change, focus, recording, etc.).

## Why the previous interface was unsuitable

The original `/siyi/set_attitude` and `/siyi/set_speed` services:

1. Block the ROS2 executor until the gimbal ACKs (5 s timeout).
2. Send absolute angles, which are quantised to 0.1° and ACK-latency
   limited.
3. Return the gimbal's attitude *at ACK time*, not the achieved
   target — useless as feedback for a controller.
4. Have no watchdog: a tracker crash leaves the gimbal slewing.

The new path replaces all four problems.

## Topic interface (control)

### `/siyi/cmd/rate` — `siyi_msgs/GimbalRateCmd` (primary)

Publish at 30–100 Hz. Rates are in degrees per second in the gimbal
frame. The wrapper converts to the SIYI -100..100 normalised range
using the configurable max-rate parameters and dispatches
**fire-and-forget** to the gimbal (`client.rotate_nowait`).

Properties:

- **Latest-wins coalescing.** Only the most recent setpoint matters;
  the wrapper never queues.
- **Watchdog.** If no command arrives within `watchdog_timeout_ms`
  (default 200 ms), a zero rate is sent and re-sent until a fresh
  command arrives. If your tracker dies the gimbal stops.
- **Soft position clamping.** If current attitude is at a mechanical
  limit and the commanded rate would push further in that direction,
  the offending axis is zeroed before dispatch.
- **Body-rate feedforward** (optional, off by default — see
  `feedforward_gain`).

### `/siyi/cmd/attitude` — `siyi_msgs/GimbalAttitudeCmd`

Absolute yaw/pitch setpoint, fire-and-forget, clamped to limits.
Use for "look at this bearing"; do not use in the inner control loop.

### `/siyi/aircraft_body_rate` — `geometry_msgs/Vector3Stamped`

Aircraft body rates in **deg/s**, expressed in the **gimbal frame**:

- `x` = roll rate (unused for now)
- `y` = pitch rate
- `z` = yaw rate

Published by your FCU bridge. The wrapper reads the latest sample
inside `_on_rate_cmd` and adds `feedforward_gain * body_rate` to the
tracker's setpoint before dispatch. With `feedforward_gain = 1.0` the
gimbal mostly stays inertially fixed under airframe motion; the visual
loop then only has to chase target motion.

> **Frame convention.** You are responsible for rotating FCU body
> rates into the gimbal frame before publishing here. The wrapper does
> not know the gimbal mounting orientation.

## Topic interface (feedback)

### `/siyi/attitude` — `siyi_msgs/GimbalAttitude`

Gimbal attitude push, default 50 Hz, **`SensorDataQoS`** (best-effort,
depth 1). Use this — not `get_gimbal_attitude()` — as your control
feedback.

### `/siyi/saturation` — `siyi_msgs/GimbalSaturation`

Published whenever the rate-command subscriber clamps an axis. When
`yaw_saturated` is true, the tracker should yaw the **airframe** to
move the limit back into range.

## Services

| Service | Type | When |
|---|---|---|
| `/siyi/tracking/enable` | `std_srvs/SetBool` | Arm/disarm visual loop. On disable, zero rate is sent immediately. |
| `/siyi/set_attitude` | `siyi_msgs/SetGimbalAttitude` | Blocking, ACK-confirmed. Setup only — never in control loop. |
| `/siyi/set_speed` | `siyi_msgs/SetGimbalSpeed` | Blocking variant; same caveat. |
| `/siyi/center` | `siyi_msgs/CenterGimbal` | One-key centering. |
| `/siyi/set_mode` | `siyi_msgs/SetGimbalMode` | Lock/follow/FPV. |

## Parameters

All parameters live on the `siyi_node`.

| Name | Default | Notes |
|---|---|---|
| `attitude_stream_hz` | `50` | Gimbal feedback rate. Set to `100` if firmware is stable on your unit. |
| `max_yaw_rate_dps` | `90.0` | Tracker rate that maps to ±100 in protocol. |
| `max_pitch_rate_dps` | `90.0` | Same, pitch axis. |
| `feedforward_gain` | `0.0` | Multiplier on body-rate FF. Start at 0.0, raise to ~1.0 once the FCU bridge + frame rotation are verified. |
| `watchdog_timeout_ms` | `200` | Max age of last rate cmd before zero is sent. Choose ≈ 4× your tracker publish period. |
| `yaw_min_deg` / `yaw_max_deg` | `-135.0` / `135.0` | Override A8 mini limits if your unit's range differs. |
| `pitch_min_deg` / `pitch_max_deg` | `-90.0` / `25.0` | Same, pitch. |
| `tracking_enabled_at_start` | `true` | If false, you must call `/siyi/tracking/enable` before commands take effect. |

## Recommended tracker design

A minimal but correct tracker for testing:

```python
# In your tracker node, no siyi_sdk dependency required.

self.rate_pub = self.create_publisher(
    GimbalRateCmd, "/siyi/cmd/rate", qos_profile_sensor_data
)

def on_detection(self, bbox):
    # Pixel error from image center.
    err_x = (bbox.cx - self.img_width / 2) / self.img_width
    err_y = (bbox.cy - self.img_height / 2) / self.img_height

    # Convert to deg/s. K_p is in "deg/s per normalised pixel error".
    yaw_rate_dps = self.k_p_yaw * err_x          # +err_x → target on right → yaw right
    pitch_rate_dps = -self.k_p_pitch * err_y     # +err_y → target below → pitch down

    msg = GimbalRateCmd()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.yaw_rate_dps = float(yaw_rate_dps)
    msg.pitch_rate_dps = float(pitch_rate_dps)
    self.rate_pub.publish(msg)
```

Publish at the detector's frame rate (30–60 Hz typically). The
watchdog will stop the gimbal if detections drop out.

### Tuning order

1. **Start with P only.** Pick `k_p` so a target at the image edge
   commands ~half the max yaw rate. Verify the gimbal moves toward
   the target and slows as it centers.
2. **Verify zero on lost detection.** Pull power on the detector
   mid-track. Gimbal should stop within `watchdog_timeout_ms`.
3. **Add integral term** only after the camera is mounted and you see
   a steady-state offset under airframe drift. Clamp the integrator;
   reset on `/siyi/saturation`.
4. **Enable feedforward last.** Publish `/siyi/aircraft_body_rate`
   from your FCU bridge in the gimbal frame, then raise
   `feedforward_gain` from 0 to ~1.0. The visual error should drop
   sharply under aggressive airframe maneuvers.

## Multi-threaded executor

The node uses `MultiThreadedExecutor` with three callback groups:

- `ReentrantCallbackGroup` for `/siyi/cmd/rate` and `/siyi/cmd/attitude`
  subscribers — they dispatch fire-and-forget and return immediately.
- `MutuallyExclusiveCallbackGroup` for the watchdog timer and
  `/siyi/tracking/enable` service — serialised but never block
  command dispatch.
- The asyncio thread (in `AsyncBridge`) owns all I/O to the gimbal.
  ROS callbacks submit coroutines and never `await`.

This means a service call to `/siyi/set_attitude` will block the
service thread for up to 5 s, but **will not** stall `/siyi/cmd/rate`
or the attitude publisher.

## What changed in the SDK

Three additions to `siyi_sdk` (see `~/src/siyi_sdk/docs/quickstart.md`
section "High-rate control"):

| Method | CMD | Notes |
|---|---|---|
| `client.rotate_nowait(yaw, pitch)` | 0x07 | Fire-and-forget velocity. Bypasses per-CMD_ID ACK lock. |
| `client.set_attitude_nowait(yaw_deg, pitch_deg)` | 0x0E | Fire-and-forget absolute attitude. |
| `client.set_single_axis_nowait(axis, angle_deg)` | 0x41 | Fire-and-forget single-axis. |

And mechanical-limit constants in `siyi_sdk.constants`:
`A8MINI_YAW_MIN_DEG`, `A8MINI_YAW_MAX_DEG`, `A8MINI_PITCH_MIN_DEG`,
`A8MINI_PITCH_MAX_DEG`, `GIMBAL_RATE_CMD_MIN`, `GIMBAL_RATE_CMD_MAX`.

Test coverage: see
`~/src/siyi_sdk/tests/test_client.py::TestGimbalCommands::test_rotate_nowait*`
and `test_set_*_nowait*`.

## Quick verification

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# Start the node.
ros2 run siyi_ros2 siyi_node

# In another shell — verify telemetry rate.
ros2 topic hz /siyi/attitude            # expect ~50 Hz

# Send a single rate setpoint.
ros2 topic pub --once /siyi/cmd/rate siyi_msgs/msg/GimbalRateCmd \
    '{yaw_rate_dps: 20.0, pitch_rate_dps: 0.0}'
# Gimbal should yaw briefly, then stop after watchdog_timeout_ms.

# Continuous setpoint at 50 Hz.
ros2 topic pub -r 50 /siyi/cmd/rate siyi_msgs/msg/GimbalRateCmd \
    '{yaw_rate_dps: 10.0, pitch_rate_dps: 5.0}'

# Disable and verify the gimbal stops.
ros2 service call /siyi/tracking/enable std_srvs/srv/SetBool '{data: false}'

# Watch for limit saturation while sweeping.
ros2 topic echo /siyi/saturation
```

## Diagnostics

The rate-command subscriber tracks two counters internally:

- `in_flight` — coroutines submitted to the asyncio loop but not yet
  completed. Should stay near 0; sustained growth means the asyncio
  loop is falling behind UDP sends.
- `dropped` — reserved for future use.

Both are returned in the response message of `/siyi/tracking/enable`,
so a quick `ros2 service call ... '{data: true}'` reports them.

## Known limitations

- The fire-and-forget UDP send has no link-level acknowledgement. A
  dropped packet on the Ethernet bridge is lost silently. At 50 Hz the
  next command arrives in ≤20 ms, so it doesn't affect tracking — but
  do not rely on individual commands taking effect.
- The wrapper does not currently forward aircraft attitude to the
  gimbal's internal stabiliser (0x22). Adding this is straightforward
  and would offload more work to the gimbal firmware.
- The saturation publisher fires on every clamped sample (no
  rate-limit). At 50 Hz, that's a 50 Hz topic when saturated. If this
  is too noisy for your logger, add a latch in your tracker.
