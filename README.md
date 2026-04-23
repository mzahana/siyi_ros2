# siyi_ros2

ROS2 Jazzy package for SIYI gimbal-camera systems (ZT30, ZT6, ZR30, ZR10, A8 Mini, A2 Mini).

Wraps the [`siyi_sdk`](https://github.com/mzahana/siyi_sdk) Python library to expose
the full SIYI command set as ROS2 topics, services, and a video streaming node.

---

## Features

- **Full command coverage** — 30+ ROS2 services for gimbal control, camera, zoom, focus, laser, thermal
- **Live telemetry** — attitude, laser distance, and AI tracking published as ROS2 topics
- **Video streaming** — RTSP H.264/H.265 → `sensor_msgs/Image` via GStreamer (lowest latency)
- **Multi-transport** — UDP (default), TCP, or Serial/UART connection
- **Auto-reconnect** — configurable reconnection on transport failure
- **ROS2 Jazzy** — tested on Ubuntu 24.04 + ROS2 Jazzy

---

## Prerequisites

| Requirement | Version |
|-------------|---------|
| ROS2 | Jazzy |
| Python | ≥ 3.10 |
| **siyi_sdk** | **must be cloned and installed — see below** |
| GStreamer (optional) | for video streaming |

---

## Installation

### Step 1 — Clone and install siyi_sdk (required)

> **This package depends on [`siyi_sdk`](https://github.com/mzahana/siyi_sdk) — you must clone it first.**
> It is not available on PyPI and will not be installed automatically.

```bash
git clone https://github.com/mzahana/siyi_sdk.git ~/src/siyi_sdk
pip install -e ~/src/siyi_sdk
```

### Step 2 — Clone the ROS2 packages

```bash
cd ~/ros2_ws/src
git clone https://github.com/mzahana/siyi_msgs.git
git clone https://github.com/mzahana/siyi_ros2.git
```

### Step 3 — Build

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --packages-select siyi_msgs siyi_ros2 --symlink-install
```

### Step 4 — Source

```bash
source ~/ros2_ws/install/setup.bash
```

---

## Quick Start

### Gimbal control node only

```bash
ros2 launch siyi_ros2 siyi.launch.py host:=192.168.144.25
```

### Full stack (gimbal + camera)

```bash
ros2 launch siyi_ros2 siyi_full.launch.py host:=192.168.144.25 camera_model:=zt30
```

### Serial connection

```bash
ros2 launch siyi_ros2 siyi.launch.py transport:=serial serial_device:=/dev/ttyUSB0
```

---

## Logging

By default the launch files suppress verbose SDK and GStreamer debug output so the
terminal only shows warnings and errors. Three environment variables control this:

| Variable | Default (in launch) | Effect |
|----------|---------------------|--------|
| `SIYI_LOG_LEVEL` | `WARNING` | SDK log level (`DEBUG` / `INFO` / `WARNING` / `ERROR`) |
| `SIYI_PROTOCOL_TRACE` | `0` | Set to `1` to log raw UDP frame hex dumps at DEBUG level |
| `GST_DEBUG` | `0` | GStreamer log verbosity (0 = silent, 3 = info, 5 = debug) |

To enable full protocol tracing for debugging:

```bash
SIYI_LOG_LEVEL=DEBUG SIYI_PROTOCOL_TRACE=1 GST_DEBUG=2 \
    ros2 launch siyi_ros2 siyi_full.launch.py host:=192.168.144.25 camera_model:=a8
```

---

## Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/siyi/attitude` | `siyi_msgs/GimbalAttitude` | Gimbal yaw/pitch/roll + rates (10 Hz) |
| `/siyi/laser_distance` | `siyi_msgs/LaserDistance` | Laser rangefinder distance (m) |
| `/siyi/ai_tracking` | `siyi_msgs/AITrackingTarget` | AI tracking target bounding box |
| `/siyi/image_raw` | `sensor_msgs/Image` | Raw video frames (BGR8) |
| `/siyi/image_compressed` | `sensor_msgs/CompressedImage` | JPEG-compressed frames |
| `/siyi/camera_info` | `sensor_msgs/CameraInfo` | Camera metadata |

---

## Services

### Gimbal

| Service | Type | Description |
|---------|------|-------------|
| `/siyi/set_attitude` | `siyi_msgs/SetGimbalAttitude` | Absolute yaw+pitch target (deg) |
| `/siyi/set_speed` | `siyi_msgs/SetGimbalSpeed` | Continuous yaw+pitch speed (−100..100) |
| `/siyi/center` | `siyi_msgs/CenterGimbal` | One-key centering |
| `/siyi/set_mode` | `siyi_msgs/SetGimbalMode` | Lock / follow / FPV mode |

### Camera

| Service | Type | Description |
|---------|------|-------------|
| `/siyi/take_photo` | `siyi_msgs/TakePhoto` | Trigger photo capture |
| `/siyi/start_recording` | `siyi_msgs/StartRecording` | Start video recording |
| `/siyi/stop_recording` | `siyi_msgs/StopRecording` | Stop video recording |
| `/siyi/set_zoom` | `siyi_msgs/SetAbsoluteZoom` | Absolute zoom level |
| `/siyi/manual_zoom` | `siyi_msgs/ManualZoom` | Zoom in/out/stop |
| `/siyi/auto_focus` | `siyi_msgs/AutoFocus` | Touch-to-focus (0–100 screen %) |
| `/siyi/manual_focus` | `siyi_msgs/ManualFocus` | Manual focus in/out/stop |
| `/siyi/set_osd` | `siyi_msgs/SetOSD` | Toggle OSD overlay |
| `/siyi/set_pseudo_color` | `siyi_msgs/SetPseudoColor` | Thermal pseudo-colour mode |

### System / Sensor

| Service | Type | Description |
|---------|------|-------------|
| `/siyi/get_firmware_version` | `siyi_msgs/GetFirmwareVersion` | Camera/gimbal/zoom firmware strings |
| `/siyi/get_gimbal_info` | `siyi_msgs/GetGimbalInfo` | Motion mode, mounting direction |
| `/siyi/set_laser_ranging` | `siyi_msgs/SetLaserRanging` | Enable/disable laser |
| `/siyi/get_laser_distance` | `siyi_msgs/GetLaserDistance` | One-shot laser measurement |

---

## Parameters

### siyi_node

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `host` | string | `192.168.144.25` | Gimbal IP address |
| `port` | int | `37260` | UDP or TCP port |
| `transport` | string | `udp` | `udp` / `tcp` / `serial` |
| `serial_device` | string | `/dev/ttyUSB0` | Serial port path |
| `baud_rate` | int | `115200` | Serial baud rate |
| `timeout` | double | `2.0` | Command timeout (s) |
| `auto_reconnect` | bool | `false` | Reconnect on failure |
| `attitude_stream_hz` | int | `10` | Attitude publish rate |

### siyi_camera_node

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `rtsp_url` | string | `""` | Override RTSP URL (auto-built if empty) |
| `camera_model` | string | `zt30` | Model key for URL generation |
| `host` | string | `192.168.144.25` | Camera IP address |
| `stream_index` | int | `0` | Stream index (0=main, 1=sub) |
| `backend` | string | `gstreamer` | `gstreamer` / `opencv` / `aiortsp` / `auto` |
| `image_encoding` | string | `bgr8` | ROS2 image encoding |
| `publish_compressed` | bool | `true` | Publish CompressedImage topic |
| `frame_id` | string | `siyi_camera` | Camera frame in TF tree |

---

## Example Usage

```bash
# Stream attitude at 20 Hz
ros2 launch siyi_ros2 siyi.launch.py attitude_stream_hz:=20

# Center gimbal
ros2 service call /siyi/center siyi_msgs/srv/CenterGimbal {}

# Point to yaw=45°, pitch=-30°
ros2 service call /siyi/set_attitude siyi_msgs/srv/SetGimbalAttitude \
    "{yaw_deg: 45.0, pitch_deg: -30.0}"

# Zoom to 5×
ros2 service call /siyi/set_zoom siyi_msgs/srv/SetAbsoluteZoom "{zoom_level: 5.0}"

# Take a photo
ros2 service call /siyi/take_photo siyi_msgs/srv/TakePhoto {}

# View live video (requires camera node)
ros2 run rqt_image_view rqt_image_view /siyi/image_raw
```

---

## Related Package

Custom message and service definitions live in a companion package:
[`siyi_msgs`](https://github.com/mzahana/siyi_msgs) — clone it alongside this package into your workspace `src/`.

---

## License

MIT — see [LICENSE](LICENSE)
