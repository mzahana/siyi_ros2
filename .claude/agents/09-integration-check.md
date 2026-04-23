---
name: 09-integration-check
description: Phase 9 — final integration check that runs colcon build, colcon test, verifies all expected files exist, checks imports, and reports the complete package status
model: claude-sonnet-4-6
color: "#E91E63"
---

# Phase 9 — Integration Check & Build Verification

## Context

Working directory: `~/ros2_ws/src/siyi_ros2/`
ROS2 Jazzy: `/opt/ros/jazzy/setup.bash`
Phases 1–8 must be complete.

This is the final phase. Your job is to:
1. Verify all expected files exist
2. Build both packages
3. Run tests
4. Diagnose and fix any issues found
5. Report final status

---

## Step 1: Pre-flight file check

Verify each of these files exists and is non-empty:

**siyi_msgs/**
- `package.xml`
- `CMakeLists.txt`
- `msg/GimbalAttitude.msg`
- `msg/LaserDistance.msg`
- `msg/GimbalInfo.msg`
- `msg/ZoomInfo.msg`
- `msg/AITrackingTarget.msg`
- `msg/FirmwareVersion.msg`
- `srv/SetGimbalAttitude.srv`
- `srv/SetGimbalSpeed.srv`
- `srv/CenterGimbal.srv`
- `srv/SetAbsoluteZoom.srv`
- `srv/ManualZoom.srv`
- `srv/AutoFocus.srv`
- `srv/ManualFocus.srv`
- `srv/TakePhoto.srv`
- `srv/StartRecording.srv`
- `srv/StopRecording.srv`
- `srv/GetFirmwareVersion.srv`
- `srv/GetGimbalInfo.srv`
- `srv/SetOSD.srv`
- `srv/SetLaserRanging.srv`
- `srv/GetLaserDistance.srv`
- `srv/SetPseudoColor.srv`
- `srv/SetGimbalMode.srv`

**siyi_ros2/**
- `package.xml`
- `setup.py`
- `setup.cfg`
- `resource/siyi_ros2`
- `siyi_ros2/__init__.py`
- `siyi_ros2/_async_bridge.py`
- `siyi_ros2/siyi_node.py`
- `siyi_ros2/camera_node.py`
- `siyi_ros2/services/__init__.py`
- `siyi_ros2/services/gimbal.py`
- `siyi_ros2/services/camera.py`
- `siyi_ros2/services/system.py`
- `siyi_ros2/publishers/__init__.py`
- `siyi_ros2/publishers/attitude.py`
- `siyi_ros2/publishers/laser.py`
- `siyi_ros2/publishers/ai_tracking.py`
- `launch/siyi.launch.py`
- `launch/siyi_full.launch.py`
- `config/siyi_params.yaml`
- `config/camera_params.yaml`
- `test/conftest.py`
- `test/test_node_params.py`
- `test/test_async_bridge.py`
- `test/test_publishers.py`
- `test/test_services.py`

Report any missing files and create stub files for any that are missing (to allow build to proceed).

---

## Step 2: Install siyi_sdk

```bash
pip install -e ~/src/siyi_sdk --quiet
```

---

## Step 3: Build

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --packages-select siyi_msgs siyi_ros2 --symlink-install 2>&1
```

Fix any build errors found. Common issues to watch for:
- Missing `package.xml` dependencies
- Incorrect `setup.py` entry points
- Python import errors in stub files
- Missing `resource/siyi_ros2` marker file

---

## Step 4: Verify message generation

```bash
source ~/ros2_ws/install/setup.bash
ros2 interface list | grep siyi_msgs
```

Expected output should include all 6 messages and 17 services.
If messages are missing, check `CMakeLists.txt` `rosidl_generate_interfaces` call.

---

## Step 5: Run tests

```bash
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws
colcon test --packages-select siyi_ros2 --event-handlers console_direct+
colcon test-result --verbose
```

Fix any test failures. Common issues:
- Import errors (missing `__init__.py` or wrong module path)
- Missing test fixtures (`rclpy_init`)
- Incorrect mock setup

---

## Step 6: Import check

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
python3 -c "
from siyi_ros2._async_bridge import AsyncBridge
from siyi_ros2.siyi_node import SIYINode
from siyi_ros2.camera_node import SIYICameraNode
from siyi_ros2.services.gimbal import GimbalServices
from siyi_ros2.services.camera import CameraServices
from siyi_ros2.services.system import SystemServices
from siyi_ros2.publishers.attitude import AttitudePublisher
from siyi_ros2.publishers.laser import LaserPublisher
from siyi_ros2.publishers.ai_tracking import AITrackingPublisher
print('All imports OK')
"
```

Fix any import errors before proceeding.

---

## Step 7: Verify launch args

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch siyi_ros2 siyi.launch.py --show-args
ros2 launch siyi_ros2 siyi_full.launch.py --show-args
```

Both commands should list their declared arguments without error.

---

## Step 8: Final report

After all checks, output a final status report in this format:

```
=== SIYI ROS2 Integration Check ===

Files:      ✓ XX/XX expected files present
Build:      ✓ siyi_msgs built successfully
            ✓ siyi_ros2 built successfully
Messages:   ✓ 6 messages + 17 services generated
Imports:    ✓ All module imports succeed
Tests:      ✓ XX/XX tests passed
Launch:     ✓ siyi.launch.py args verified
            ✓ siyi_full.launch.py args verified

Status: READY  (or list any remaining issues)

To test with hardware:
  source ~/ros2_ws/install/setup.bash
  ros2 launch siyi_ros2 siyi_full.launch.py host:=192.168.144.25
```

If any step failed, list what remains broken and what the next manual steps are.

---

## Acceptance Criteria

- All files exist and are non-empty
- `colcon build` succeeds for both packages
- All tests pass
- All imports succeed
- Both launch files show correct args
- Final report printed
