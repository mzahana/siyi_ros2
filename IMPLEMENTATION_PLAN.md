# SIYI ROS2 Package — Implementation Plan

## Overview

This plan describes the engineering work to build `siyi_ros2`, a professional ROS2 Jazzy
package that wraps the [`siyi_sdk`](https://github.com/mabdelkader/siyi_sdk) Python library
to expose SIYI gimbal-camera systems as first-class ROS2 components.

The workspace is at `~/ros2_ws/` (ROS2 Jazzy).
The upstream SDK is at `~/src/siyi_sdk` (Python ≥ 3.10, fully async/asyncio).

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│  ROS2 Process                                                        │
│                                                                      │
│  ┌──────────────────────────┐    ┌───────────────────────────────┐  │
│  │   SIYINode (rclpy.Node)  │    │  SIYICameraNode (rclpy.Node)  │  │
│  │                          │    │                               │  │
│  │  • /siyi/attitude        │    │  • /siyi/image_raw            │  │
│  │  • /siyi/laser_distance  │    │  • /siyi/camera_info          │  │
│  │  • /siyi/gimbal_info     │    │  • /siyi/compressed           │  │
│  │  • /siyi/ai_tracking     │    │                               │  │
│  │  • Services (30+)        │    │  Uses SIYIStream (GStreamer)   │  │
│  │                          │    │                               │  │
│  └──────────┬───────────────┘    └────────────┬──────────────────┘  │
│             │  asyncio.run_coroutine_threadsafe│                      │
│  ┌──────────▼─────────────────────────────────▼──────────────────┐  │
│  │  asyncio event loop  (dedicated daemon thread)                 │  │
│  │                                                                │  │
│  │   SIYIClient  ◄──── connect_udp / connect_tcp / connect_serial │  │
│  │   (siyi_sdk)                                                   │  │
│  └────────────────────────────────────────────────────────────────┘  │
│                             │ UDP/TCP/Serial                          │
└─────────────────────────────┼───────────────────────────────────────┘
                              │
                       SIYI Gimbal/Camera
```

### Threading Model

| Thread | Role |
|--------|------|
| Main (ROS2 executor) | `rclpy.spin()`, service callbacks, timer callbacks |
| asyncio thread | `asyncio event loop` — runs `SIYIClient` I/O, parser, heartbeat |

**Bridge**: service/timer callbacks call `asyncio.run_coroutine_threadsafe(coro, loop).result(timeout)`.
**Callbacks from SDK → ROS2**: `on_attitude`, `on_laser_distance` etc. are registered and call
`publisher.publish()` which is thread-safe in ROS2.

---

## Repository Layout

```
siyi_ros2/                         ← repo root (this folder)
├── .claude/agents/                ← implementation agents (run in order)
├── IMPLEMENTATION_PLAN.md
├── README.md
├── CLAUDE.md
│
├── siyi_msgs/                     ← ROS2 package: message & service definitions
│   ├── package.xml                  (ament_cmake)
│   ├── CMakeLists.txt
│   ├── msg/
│   │   ├── GimbalAttitude.msg
│   │   ├── LaserDistance.msg
│   │   ├── GimbalInfo.msg
│   │   ├── ZoomInfo.msg
│   │   ├── AITrackingTarget.msg
│   │   └── FirmwareVersion.msg
│   └── srv/
│       ├── SetGimbalAttitude.srv
│       ├── SetGimbalSpeed.srv
│       ├── CenterGimbal.srv
│       ├── SetAbsoluteZoom.srv
│       ├── ManualZoom.srv
│       ├── AutoFocus.srv
│       ├── ManualFocus.srv
│       ├── TakePhoto.srv
│       ├── StartRecording.srv
│       ├── StopRecording.srv
│       ├── GetFirmwareVersion.srv
│       ├── GetGimbalInfo.srv
│       ├── SetOSD.srv
│       ├── SetLaserRanging.srv
│       ├── GetLaserDistance.srv
│       ├── SetPseudoColor.srv
│       └── SetGimbalMode.srv
│
└── siyi_ros2/                     ← ROS2 package: Python nodes
    ├── package.xml                  (ament_python)
    ├── setup.py
    ├── setup.cfg
    ├── resource/siyi_ros2
    ├── siyi_ros2/
    │   ├── __init__.py
    │   ├── siyi_node.py           ← main gimbal/camera control node
    │   ├── camera_node.py         ← RTSP video streaming node
    │   ├── _async_bridge.py       ← asyncio↔ROS2 thread bridge
    │   ├── services/
    │   │   ├── __init__.py
    │   │   ├── gimbal.py          ← gimbal rotation / attitude services
    │   │   ├── camera.py          ← photo, record, zoom, focus services
    │   │   └── system.py          ← firmware, info, reboot services
    │   └── publishers/
    │       ├── __init__.py
    │       ├── attitude.py        ← GimbalAttitude publisher
    │       ├── laser.py           ← LaserDistance publisher
    │       └── ai_tracking.py     ← AITrackingTarget publisher
    ├── launch/
    │   ├── siyi.launch.py         ← gimbal node launch
    │   └── siyi_full.launch.py    ← gimbal + camera node launch
    ├── config/
    │   ├── siyi_params.yaml
    │   └── camera_params.yaml
    └── test/
        ├── test_node_params.py
        ├── test_publishers.py
        └── test_services.py
```

---

## Implementation Phases

### Phase 1 — Package Scaffolding
**Agent**: `01-scaffolding` | **Model**: `claude-haiku-4-5-20251001`

- Create `siyi_msgs/` ROS2 package skeleton (ament_cmake, rosidl)
- Create `siyi_ros2/` Python ROS2 package skeleton (ament_python)
- Wire up `package.xml` dependencies, `CMakeLists.txt`, `setup.py`
- Create empty `__init__.py` stubs and module directories

### Phase 2 — Message & Service Definitions
**Agent**: `02-messages-services` | **Model**: `claude-haiku-4-5-20251001`

- Write all `.msg` files (GimbalAttitude, LaserDistance, GimbalInfo, ZoomInfo, AITrackingTarget, FirmwareVersion)
- Write all `.srv` files (16 services covering gimbal, camera, system)
- Update `CMakeLists.txt` to generate all interfaces

### Phase 3 — Async Bridge & Connection Node
**Agent**: `03-connection-node` | **Model**: `claude-sonnet-4-6`

- Implement `_async_bridge.py`: asyncio loop in daemon thread, `run_async()` helper
- Implement `siyi_node.py`: `SIYINode(rclpy.Node)` — parameter declaration, connection
  lifecycle, reconnection, shutdown
- ROS2 parameters: `host`, `port`, `transport` (udp/tcp/serial), `serial_device`,
  `baud_rate`, `timeout`, `auto_reconnect`, `attitude_stream_hz`

### Phase 4 — Gimbal & Camera Services
**Agent**: `04-gimbal-services` | **Model**: `claude-sonnet-4-6`

- `services/gimbal.py`: `SetGimbalAttitude`, `SetGimbalSpeed`, `CenterGimbal`, `SetGimbalMode`
- `services/camera.py`: `TakePhoto`, `StartRecording`, `StopRecording`, `SetAbsoluteZoom`,
  `ManualZoom`, `AutoFocus`, `ManualFocus`, `SetOSD`, `SetPseudoColor`
- `services/system.py`: `GetFirmwareVersion`, `GetGimbalInfo`, `SetLaserRanging`, `GetLaserDistance`
- All handlers call `self._bridge.run_async(coro)` and return typed responses

**SDK findings (discovered during implementation):**
- There is no `set_gimbal_mode` SDK command. Gimbal mode is set via `capture(CaptureFuncType)`:
  `LOCK_MODE=3`, `FOLLOW_MODE=4`, `FPV_MODE=5`.
- Recording start and stop both use `CaptureFuncType.START_RECORD` (value 2) — the SDK toggles
  on/off with the same command (confirmed from `examples/camera_capture.py`). There are no
  separate `RECORD_ON` / `RECORD_OFF` variants.
- `get_gimbal_system_info()` may not exist; use `get_camera_system_info()` for camera/zoom info.

### Phase 5 — Telemetry Publishers
**Agent**: `05-telemetry-publishers` | **Model**: `claude-haiku-4-5-20251001`

- `publishers/attitude.py`: subscribes `client.on_attitude()`, publishes `siyi_msgs/GimbalAttitude`
  with stamped header
- `publishers/laser.py`: subscribes `client.on_laser_distance()`, publishes
  `siyi_msgs/LaserDistance`
- `publishers/ai_tracking.py`: subscribes `client.on_ai_tracking()`, publishes
  `siyi_msgs/AITrackingTarget`
- Register all publishers in `SIYINode.__init__` after connection

### Phase 6 — Video Streaming Node
**Agent**: `06-video-streaming` | **Model**: `claude-sonnet-4-6`

- `camera_node.py`: standalone `SIYICameraNode(rclpy.Node)`
- RTSP URL construction from parameters (camera model, IP, stream index)
- Uses `SIYIStream` with GStreamer backend (lowest latency ~120 ms)
- Publishes: `sensor_msgs/Image` (BGR8), `sensor_msgs/CompressedImage` (jpeg),
  `sensor_msgs/CameraInfo`
- Parameters: `rtsp_url`, `camera_model`, `stream_index`, `backend`, `image_encoding`,
  `publish_compressed`, `frame_id`

### Phase 7 — Launch Files & Configuration
**Agent**: `07-launch-config` | **Model**: `claude-haiku-4-5-20251001`

- `siyi.launch.py`: launches `siyi_node` with YAML params, namespace support
- `siyi_full.launch.py`: launches both `siyi_node` and `siyi_camera_node`
- `siyi_params.yaml`: all gimbal node defaults
- `camera_params.yaml`: all camera node defaults
- `setup.py` data_files entries for launch/ and config/

### Phase 8 — Tests
**Agent**: `08-tests` | **Model**: `claude-sonnet-4-6`

- `test_node_params.py`: verify parameter defaults, overrides, validation
- `test_publishers.py`: inject mock attitude/laser data, assert published messages
- `test_services.py`: call services against mock transport, assert correct SDK calls
- Uses `siyi_sdk`'s `MockTransport` to avoid real hardware
- Uses `pytest` and `launch_testing` (ROS2 Jazzy style)

### Phase 9 — Integration Check & Build Verification
**Agent**: `09-integration-check` | **Model**: `claude-sonnet-4-6`

- Verify all expected files exist and are non-empty
- Run `colcon build --packages-select siyi_msgs siyi_ros2`
- Run `colcon test --packages-select siyi_ros2`
- Report any missing imports, unresolved symbols, or test failures
- Final checklist validation against this plan

---

## Key SDK Facts (for agents)

```python
# siyi_sdk is at ~/src/siyi_sdk
# pip install -e ~/src/siyi_sdk  (or add to PATH)

from siyi_sdk import connect_udp, connect_tcp, connect_serial, SIYIClient
from siyi_sdk.stream import SIYIStream, StreamConfig, StreamBackend

# Connection
client = await connect_udp(ip="192.168.144.25", port=37260)
client = await connect_tcp(ip="192.168.144.25", port=37260)
client = await connect_serial(device="/dev/ttyUSB0", baud=115200)
await client.close()

# Key async commands
attitude   = await client.get_gimbal_attitude()   # GimbalAttitude: yaw_deg, pitch_deg, roll_deg, rates
ack        = await client.set_attitude(yaw_deg, pitch_deg)  # SetAttitudeAck
await client.rotate(yaw_speed_int, pitch_speed_int)         # -100..100
await client.one_key_centering()                            # CenteringAction.CENTER
zoom       = await client.get_current_zoom()                # float
await client.absolute_zoom(zoom_level: float)
await client.auto_focus(touch_x, touch_y)                  # screen coords 0–100
await client.manual_focus(direction: int)                   # 1=in, -1=out, 0=stop
await client.capture(func)                                  # CaptureFuncType
fw         = await client.get_firmware_version()            # FirmwareVersion (camera/gimbal/zoom uint32)
info       = await client.get_gimbal_system_info()          # GimbalSystemInfo
cam_info   = await client.get_camera_system_info()          # CameraSystemInfo
laser      = await client.get_laser_distance()              # LaserDistance (distance_m: float | None)
await client.set_laser_ranging_state(on=True)

# Stream subscriptions (thread-safe callbacks)
unsub = client.on_attitude(callback: Callable[[GimbalAttitude], None]) -> Unsubscribe
unsub = client.on_laser_distance(callback: Callable[[LaserDistance], None]) -> Unsubscribe
unsub = client.on_ai_tracking(callback: Callable[[AITrackingTarget], None]) -> Unsubscribe

# Activate gimbal attitude stream at given Hz
from siyi_sdk.models import GimbalDataType, DataStreamFreq
await client.request_gimbal_stream(GimbalDataType.ATTITUDE, DataStreamFreq.FREQ_10HZ)

# Video streaming
from siyi_sdk.stream import SIYIStream, StreamConfig, StreamBackend
config = StreamConfig(url="rtsp://192.168.144.25:8554/main.264", backend=StreamBackend.GSTREAMER)
stream = SIYIStream(config)
unsub = stream.on_frame(callback: Callable[[StreamFrame], None])
await stream.start()   # StreamFrame.frame is np.ndarray (BGR)
await stream.stop()
```

---

## Build & Test Commands

```bash
# 1. Install siyi_sdk (one-time)
pip install -e ~/src/siyi_sdk

# 2. Source ROS2 Jazzy
source /opt/ros/jazzy/setup.bash

# 3. Build
cd ~/ros2_ws
colcon build --packages-select siyi_msgs siyi_ros2 --symlink-install

# 4. Source workspace
source install/setup.bash

# 5. Test
colcon test --packages-select siyi_ros2
colcon test-result --verbose

# 6. Run (hardware connected at 192.168.144.25)
ros2 launch siyi_ros2 siyi_full.launch.py host:=192.168.144.25
```

---

## Agent Execution Order

Agents **must** be run in order (each depends on the previous):

```
01 → 02 → 03 → 04 → 05 → 06 → 07 → 08 → 09
```

See [CLAUDE.md](CLAUDE.md) for exact invocation instructions.
