---
name: 04-gimbal-services
description: Phase 4 — implements all ROS2 service handlers for gimbal control, camera commands, zoom/focus, and system info, wired into SIYINode
model: claude-sonnet-4-6
color: "#9B59B6"
---

# Phase 4 — Gimbal, Camera & System Services

## Context

Working directory: `~/ros2_ws/src/siyi_ros2/`
siyi_sdk location: `~/src/siyi_sdk`
Phases 1–3 must be complete.

You are implementing all ROS2 service handlers and wiring them into `SIYINode._setup_services()`.

Read the siyi_sdk source before writing:
- `~/src/siyi_sdk/siyi_sdk/client.py` — exact method signatures
- `~/src/siyi_sdk/siyi_sdk/models.py` — return type fields

---

## Pattern for all service handlers

Every handler follows this pattern:

```python
def _handle_XXXX(
    self,
    request: XxxxSrv.Request,
    response: XxxxSrv.Response,
) -> XxxxSrv.Response:
    try:
        result = self._bridge.run_async(
            self._client.some_method(...), timeout=5.0
        )
        response.success = True
        response.message = "OK"
        # fill response fields from result
    except TimeoutError:
        response.success = False
        response.message = "Timeout waiting for SIYI response"
    except Exception as exc:
        response.success = False
        response.message = str(exc)
        self._node.get_logger().error(f"Service call failed: {exc}")
    return response
```

All handlers are **free functions or class methods** that receive `self._node` and
`self._bridge` and `self._client` — implement each module as a class
(`GimbalServices`, `CameraServices`, `SystemServices`) that is instantiated in
`SIYINode._setup_services()` and stores a reference to the node.

---

## File 1: `siyi_ros2/siyi_ros2/services/gimbal.py`

Class `GimbalServices` — registers these services on the node:

### `/siyi/set_attitude` → `SetGimbalAttitude`
- Calls: `await client.set_attitude(yaw_deg=req.yaw_deg, pitch_deg=req.pitch_deg)`
- Returns `SetAttitudeAck`: fields `yaw_deg`, `pitch_deg`, `roll_deg`
- Fill response: `actual_yaw_deg`, `actual_pitch_deg`, `actual_roll_deg`

### `/siyi/set_speed` → `SetGimbalSpeed`
- Calls: `await client.rotate(yaw=req.yaw_speed, pitch=req.pitch_speed)`
- No return value (void) — set `success=True` if no exception

### `/siyi/center` → `CenterGimbal`
- Calls: `await client.one_key_centering()`
- No return value — set `success=True`

### `/siyi/set_mode` → `SetGimbalMode`
- Map `req.mode` (uint8: 0=lock, 1=follow, 2=fpv) to `GimbalMotionMode` enum
- Calls: `await client.get_gimbal_mode()` to verify after set
- Note: check if siyi_sdk has a `set_gimbal_mode` command; if not, note it in response message
  and return `success=False, message="set_mode not supported by SDK"`

```python
from siyi_msgs.srv import SetGimbalAttitude, SetGimbalSpeed, CenterGimbal, SetGimbalMode
from siyi_sdk.models import GimbalMotionMode

class GimbalServices:
    def __init__(self, node, bridge, client):
        self._node = node
        self._bridge = bridge
        self._client = client
        node.create_service(SetGimbalAttitude, '/siyi/set_attitude', self._handle_set_attitude)
        node.create_service(SetGimbalSpeed, '/siyi/set_speed', self._handle_set_speed)
        node.create_service(CenterGimbal, '/siyi/center', self._handle_center)
        node.create_service(SetGimbalMode, '/siyi/set_mode', self._handle_set_mode)
```

---

## File 2: `siyi_ros2/siyi_ros2/services/camera.py`

Class `CameraServices` — registers these services:

### `/siyi/take_photo` → `TakePhoto`
- Calls: `await client.capture(CaptureFuncType.PHOTO)` 
- Import: `from siyi_sdk.models import CaptureFuncType`

### `/siyi/start_recording` → `StartRecording`
- Calls: `await client.capture(CaptureFuncType.RECORD_ON)`

### `/siyi/stop_recording` → `StopRecording`
- Calls: `await client.capture(CaptureFuncType.RECORD_OFF)`

### `/siyi/set_zoom` → `SetAbsoluteZoom`
- Calls: `await client.absolute_zoom(req.zoom_level)`
- Then: `actual = await client.get_current_zoom()`
- Fill: `response.actual_zoom = actual`

### `/siyi/manual_zoom` → `ManualZoom`
- Calls: `await client.manual_zoom(req.direction)`
- Returns current zoom (float) — fill `response.current_zoom`

### `/siyi/auto_focus` → `AutoFocus`
- Calls: `await client.auto_focus(touch_x=int(req.touch_x), touch_y=int(req.touch_y))`
- `auto_focus` takes integer screen coords 0–100

### `/siyi/manual_focus` → `ManualFocus`
- Calls: `await client.manual_focus(req.direction)`

### `/siyi/set_osd` → `SetOSD`
- Calls: `await client.set_osd_flag(req.enabled)`
- Returns bool — set `success` based on return value

### `/siyi/set_pseudo_color` → `SetPseudoColor`
- Map `req.color_mode` (uint8) to `PseudoColor` enum
- Calls: `await client.set_pseudo_color(color)`
- Returns `PseudoColor` enum — fill `response.actual_color_mode = result.value`
- Import: `from siyi_sdk.models import PseudoColor`

```python
from siyi_msgs.srv import (
    TakePhoto, StartRecording, StopRecording, SetAbsoluteZoom,
    ManualZoom, AutoFocus, ManualFocus, SetOSD, SetPseudoColor,
)
from siyi_sdk.models import CaptureFuncType, PseudoColor

class CameraServices:
    def __init__(self, node, bridge, client): ...
```

---

## File 3: `siyi_ros2/siyi_ros2/services/system.py`

Class `SystemServices` — registers these services:

### `/siyi/get_firmware_version` → `GetFirmwareVersion`
- Calls: `await client.get_firmware_version()`
- Returns `FirmwareVersion` with fields: `camera`, `gimbal`, `zoom` (uint32 encoded)
- Use `FirmwareVersion.format_word(fw.camera)` to get human-readable strings
- Fill: `response.camera_version`, `response.gimbal_version`, `response.zoom_version`

### `/siyi/get_gimbal_info` → `GetGimbalInfo`
- Calls: `await client.get_camera_system_info()`
- Returns `CameraSystemInfo` with fields:
  - `record_sta: RecordingState`
  - `gimbal_motion_mode: GimbalMotionMode`
  - `gimbal_mounting_dir: MountingDirection`
  - `hdr_sta: int`
  - `zoom_linkage: int`
- Also calls: `await client.get_current_zoom()` for zoom_level
- Build `siyi_msgs/GimbalInfo` message and fill response

### `/siyi/set_laser_ranging` → `SetLaserRanging`
- Calls: `await client.set_laser_ranging_state(req.enabled)`
- Returns bool

### `/siyi/get_laser_distance` → `GetLaserDistance`
- Calls: `await client.get_laser_distance()`
- Returns `LaserDistance` with field `distance_m: float | None`
- Fill: `response.distance_m = laser.distance_m or 0.0`
- Fill: `response.valid = laser.distance_m is not None`

```python
from siyi_msgs.srv import GetFirmwareVersion, GetGimbalInfo, SetLaserRanging, GetLaserDistance
from siyi_msgs.msg import GimbalInfo
from siyi_sdk.models import FirmwareVersion

class SystemServices:
    def __init__(self, node, bridge, client): ...
```

---

## Wire into SIYINode

Edit `siyi_ros2/siyi_ros2/siyi_node.py` to replace the `_setup_services` stub:

```python
def _setup_services(self) -> None:
    from siyi_ros2.services.gimbal import GimbalServices
    from siyi_ros2.services.camera import CameraServices
    from siyi_ros2.services.system import SystemServices
    self._gimbal_services = GimbalServices(self, self._bridge, self._client)
    self._camera_services = CameraServices(self, self._bridge, self._client)
    self._system_services = SystemServices(self, self._bridge, self._client)
```

---

## Acceptance Criteria

- All three service modules exist with their respective classes
- Each class registers the correct services in `__init__`
- Handler pattern: `try/except TimeoutError/Exception`, always returns response
- `colcon build --packages-select siyi_ros2` succeeds
- Running `ros2 service list` after launch shows all `/siyi/*` services
- No bare `except:` — use `TimeoutError` then `Exception`
