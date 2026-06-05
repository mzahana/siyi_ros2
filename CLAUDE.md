# SIYI ROS2 Package — Claude Code Context

## What this repo is

A ROS2 Jazzy package that wraps the `siyi_sdk` Python library to expose SIYI
gimbal-camera systems as ROS2 topics and services.

- **Repo root**: `~/ros2_ws/src/siyi_ros2/`
- **siyi_sdk location**: `~/src/siyi_sdk` (Python ≥ 3.10, fully async)
- **ROS2**: Jazzy on Ubuntu 24.04 (`/opt/ros/jazzy/setup.bash`)
- **Workspace**: `~/ros2_ws/`

## Sub-packages

| Package | Build type | Purpose |
|---------|-----------|---------|
| `siyi_msgs/` | ament_cmake | Custom `.msg` and `.srv` definitions |
| `siyi_ros2/` | ament_python | Python ROS2 nodes |

## Implementation agents

Run agents **in order** by invoking them in a new Claude Code session opened in this directory.

```
01 → 02 → 03 → 04 → 05 → 06 → 07 → 08 → 09
```

### How to invoke an agent

In the Claude Code chat, type:

```
Use the agent @01-scaffolding to run Phase 1.
```

Or simply describe the task and Claude Code will auto-select the matching agent based
on its description. You can also manually invoke by name:

```
@02-messages-services    run Phase 2
@03-connection-node      run Phase 3
@04-gimbal-services      run Phase 4
@05-telemetry-publishers run Phase 5
@06-video-streaming      run Phase 6
@07-launch-config        run Phase 7
@08-tests                run Phase 8
@09-integration-check    run Phase 9 (build + verify)
```

### Verify build after each phase

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --packages-select siyi_msgs siyi_ros2 --symlink-install 2>&1 | tail -20
```

## Key SDK API (quick reference)

```python
from siyi_sdk import connect_udp, connect_tcp, connect_serial
from siyi_sdk.models import GimbalDataType, DataStreamFreq

client = await connect_udp("192.168.144.25")
await client.close()

# Subscriptions
unsub = client.on_attitude(cb)           # GimbalAttitude pushed ~10 Hz
unsub = client.on_laser_distance(cb)     # LaserDistance pushed on measure
unsub = client.on_ai_tracking(cb)        # AITrackingTarget pushed when tracking

# Activate stream
await client.request_gimbal_stream(GimbalDataType.ATTITUDE, DataStreamFreq.FREQ_10HZ)
```

## Dynamic TF broadcasting

`siyi_node` can publish a dynamic `tf2` transform that encodes the gimbal's
live orientation.  It is **disabled by default** (`publish_tf: false`).

### How it works

On every attitude callback the `AttitudePublisher` converts the SIYI
yaw/pitch/roll (degrees, ZYX aerospace convention) to a quaternion and
broadcasts:

```
tf_parent_frame  →  tf_child_frame
  (e.g. base_link)     (e.g. siyi_gimbal)
```

The translation component is always zero — the transform is pure rotation.
The physical offset of the gimbal mount on the vehicle should be expressed
separately with a `static_transform_publisher` node:

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0.1 --y 0.0 --z -0.05 \
  --frame-id base_link --child-frame-id siyi_gimbal_mount
```

Then set `tf_parent_frame: "siyi_gimbal_mount"` in your params file so the
dynamic TF starts from the mount frame.

### Enabling via config file

In `config/siyi_params.yaml`:

```yaml
siyi_node:
  ros__parameters:
    publish_tf: true
    tf_parent_frame: "base_link"   # change to your vehicle body frame
    tf_child_frame: "siyi_gimbal"
```

### Enabling via launch argument

```bash
ros2 launch siyi_ros2 siyi.launch.py publish_tf:=true
ros2 launch siyi_ros2 siyi.launch.py publish_tf:=true tf_parent_frame:=body tf_child_frame:=camera_gimbal
```

### Verifying

```bash
ros2 run tf2_tools tf2_echo base_link siyi_gimbal
```

## Threading model

```
Main thread  → rclpy.spin()  (ROS2 executor + service/timer callbacks)
Asyncio thread → asyncio event loop  (SIYIClient I/O)
Bridge: asyncio.run_coroutine_threadsafe(coro, loop).result(timeout)
```
