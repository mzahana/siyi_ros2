---
name: 02-messages-services
description: Phase 2 — writes all .msg and .srv files in siyi_msgs for gimbal attitude, laser distance, zoom info, AI tracking, firmware version, and all gimbal/camera/system services
model: claude-haiku-4-5-20251001
color: "#27AE60"
---

# Phase 2 — Message & Service Definitions

## Context

Working directory: `~/ros2_ws/src/siyi_ros2/`
Package: `siyi_msgs/`
Phase 1 must be complete (directories exist, CMakeLists.txt lists these files).

Write every `.msg` and `.srv` file. The CMakeLists.txt already lists all of them.

---

## Message Files (`siyi_msgs/msg/`)

### `GimbalAttitude.msg`
```
std_msgs/Header header
float32 yaw_deg
float32 pitch_deg
float32 roll_deg
float32 yaw_rate_dps
float32 pitch_rate_dps
float32 roll_rate_dps
```

### `LaserDistance.msg`
```
std_msgs/Header header
float32 distance_m
bool valid
```

### `GimbalInfo.msg`
```
std_msgs/Header header
# motion_mode: 0=lock, 1=follow, 2=fpv
uint8 motion_mode
# mounting_dir: 0=normal, 1=upside_down
uint8 mounting_dir
bool recording
bool hdr_enabled
float32 zoom_level
```

### `ZoomInfo.msg`
```
std_msgs/Header header
float32 zoom_level
float32 max_zoom
```

### `AITrackingTarget.msg`
```
std_msgs/Header header
bool tracking
float32 target_x
float32 target_y
float32 target_w
float32 target_h
```

### `FirmwareVersion.msg`
```
string camera_version
string gimbal_version
string zoom_version
```

---

## Service Files (`siyi_msgs/srv/`)

### `SetGimbalAttitude.srv`
```
float32 yaw_deg
float32 pitch_deg
---
bool success
string message
float32 actual_yaw_deg
float32 actual_pitch_deg
float32 actual_roll_deg
```

### `SetGimbalSpeed.srv`
```
# speed range: -100 to 100 (0 = stop)
int32 yaw_speed
int32 pitch_speed
---
bool success
string message
```

### `CenterGimbal.srv`
```
---
bool success
string message
```

### `SetAbsoluteZoom.srv`
```
float32 zoom_level
---
bool success
string message
float32 actual_zoom
```

### `ManualZoom.srv`
```
# direction: 1=zoom_in, -1=zoom_out, 0=stop
int32 direction
---
bool success
string message
float32 current_zoom
```

### `AutoFocus.srv`
```
# touch position as screen percentage 0-100
float32 touch_x
float32 touch_y
---
bool success
string message
```

### `ManualFocus.srv`
```
# direction: 1=focus_in, -1=focus_out, 0=stop
int32 direction
---
bool success
string message
```

### `TakePhoto.srv`
```
---
bool success
string message
```

### `StartRecording.srv`
```
---
bool success
string message
```

### `StopRecording.srv`
```
---
bool success
string message
```

### `GetFirmwareVersion.srv`
```
---
bool success
string message
string camera_version
string gimbal_version
string zoom_version
```

### `GetGimbalInfo.srv`
```
---
bool success
string message
siyi_msgs/GimbalInfo info
```

### `SetOSD.srv`
```
bool enabled
---
bool success
string message
```

### `SetLaserRanging.srv`
```
bool enabled
---
bool success
string message
```

### `GetLaserDistance.srv`
```
---
bool success
string message
float32 distance_m
bool valid
```

### `SetPseudoColor.srv`
```
# color_mode: 0=white_hot, 1=black_hot, 2=fusion, 3=rainbow, 4=ironbow
uint8 color_mode
---
bool success
string message
uint8 actual_color_mode
```

### `SetGimbalMode.srv`
```
# mode: 0=lock, 1=follow, 2=fpv
uint8 mode
---
bool success
string message
```

---

## Acceptance Criteria

- All 6 `.msg` files and 17 `.srv` files exist in `siyi_msgs/msg/` and `siyi_msgs/srv/`
- No `.gitkeep` files remaining in those directories
- `colcon build --packages-select siyi_msgs` succeeds and generates interfaces
- Run: `ros2 interface list | grep siyi_msgs` shows all messages and services
