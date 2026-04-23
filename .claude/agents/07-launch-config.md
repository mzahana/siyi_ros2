---
name: 07-launch-config
description: Phase 7 — creates ROS2 Jazzy launch files (siyi.launch.py and siyi_full.launch.py) and YAML parameter configuration files for both nodes
model: claude-haiku-4-5-20251001
color: "#F1C40F"
---

# Phase 7 — Launch Files & Configuration

## Context

Working directory: `~/ros2_ws/src/siyi_ros2/`
ROS2 version: Jazzy
Phases 1–6 must be complete.

Create launch files and YAML config files for `siyi_ros2`.

---

## File 1: `siyi_ros2/config/siyi_params.yaml`

```yaml
siyi_node:
  ros__parameters:
    host: "192.168.144.25"
    port: 37260
    transport: "udp"          # udp | tcp | serial
    serial_device: "/dev/ttyUSB0"
    baud_rate: 115200
    timeout: 2.0
    auto_reconnect: false
    attitude_stream_hz: 10
```

---

## File 2: `siyi_ros2/config/camera_params.yaml`

```yaml
siyi_camera_node:
  ros__parameters:
    rtsp_url: ""               # if empty, auto-built from host + camera_model
    camera_model: "zt30"       # zt30 | zt6 | zr30 | zr10 | a8 | a2 | r1m
    host: "192.168.144.25"
    stream_index: 0
    backend: "gstreamer"       # gstreamer | opencv | aiortsp | auto
    image_encoding: "bgr8"
    publish_compressed: true
    frame_id: "siyi_camera"
```

---

## File 3: `siyi_ros2/launch/siyi.launch.py`

Launch the gimbal/control node only with YAML params + command-line overrides.

```python
"""Launch siyi_node (gimbal control, telemetry, services)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("siyi_ros2")

    # Declare overridable arguments
    args = [
        DeclareLaunchArgument("host", default_value="192.168.144.25",
                              description="SIYI gimbal IP address"),
        DeclareLaunchArgument("port", default_value="37260",
                              description="UDP or TCP port"),
        DeclareLaunchArgument("transport", default_value="udp",
                              description="Transport type: udp | tcp | serial"),
        DeclareLaunchArgument("serial_device", default_value="/dev/ttyUSB0",
                              description="Serial device path"),
        DeclareLaunchArgument("baud_rate", default_value="115200",
                              description="Serial baud rate"),
        DeclareLaunchArgument("attitude_stream_hz", default_value="10",
                              description="Attitude publish rate (Hz)"),
        DeclareLaunchArgument("auto_reconnect", default_value="false",
                              description="Auto-reconnect on failure"),
        DeclareLaunchArgument("namespace", default_value="",
                              description="Node namespace"),
    ]

    params_file = PathJoinSubstitution([pkg_share, "config", "siyi_params.yaml"])

    siyi_node = Node(
        package="siyi_ros2",
        executable="siyi_node",
        name="siyi_node",
        namespace=LaunchConfiguration("namespace"),
        parameters=[
            params_file,
            {
                "host": LaunchConfiguration("host"),
                "port": LaunchConfiguration("port"),
                "transport": LaunchConfiguration("transport"),
                "serial_device": LaunchConfiguration("serial_device"),
                "baud_rate": LaunchConfiguration("baud_rate"),
                "attitude_stream_hz": LaunchConfiguration("attitude_stream_hz"),
                "auto_reconnect": LaunchConfiguration("auto_reconnect"),
            },
        ],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription(args + [siyi_node])
```

---

## File 4: `siyi_ros2/launch/siyi_full.launch.py`

Launch both `siyi_node` and `siyi_camera_node`.

```python
"""Launch siyi_node + siyi_camera_node (full stack: control + video streaming)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("siyi_ros2")

    args = [
        DeclareLaunchArgument("host", default_value="192.168.144.25",
                              description="SIYI device IP address"),
        DeclareLaunchArgument("port", default_value="37260"),
        DeclareLaunchArgument("transport", default_value="udp"),
        DeclareLaunchArgument("attitude_stream_hz", default_value="10"),
        DeclareLaunchArgument("auto_reconnect", default_value="false"),
        DeclareLaunchArgument("camera_model", default_value="zt30",
                              description="Camera model key for RTSP URL"),
        DeclareLaunchArgument("stream_index", default_value="0"),
        DeclareLaunchArgument("backend", default_value="gstreamer",
                              description="Video backend: gstreamer | opencv | aiortsp"),
        DeclareLaunchArgument("publish_compressed", default_value="true"),
        DeclareLaunchArgument("namespace", default_value=""),
    ]

    gimbal_params = PathJoinSubstitution([pkg_share, "config", "siyi_params.yaml"])
    camera_params = PathJoinSubstitution([pkg_share, "config", "camera_params.yaml"])

    siyi_node = Node(
        package="siyi_ros2",
        executable="siyi_node",
        name="siyi_node",
        namespace=LaunchConfiguration("namespace"),
        parameters=[
            gimbal_params,
            {
                "host": LaunchConfiguration("host"),
                "port": LaunchConfiguration("port"),
                "transport": LaunchConfiguration("transport"),
                "attitude_stream_hz": LaunchConfiguration("attitude_stream_hz"),
                "auto_reconnect": LaunchConfiguration("auto_reconnect"),
            },
        ],
        output="screen",
        emulate_tty=True,
    )

    camera_node = Node(
        package="siyi_ros2",
        executable="siyi_camera_node",
        name="siyi_camera_node",
        namespace=LaunchConfiguration("namespace"),
        parameters=[
            camera_params,
            {
                "host": LaunchConfiguration("host"),
                "camera_model": LaunchConfiguration("camera_model"),
                "stream_index": LaunchConfiguration("stream_index"),
                "backend": LaunchConfiguration("backend"),
                "publish_compressed": LaunchConfiguration("publish_compressed"),
            },
        ],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription(args + [siyi_node, camera_node])
```

---

## Verify `setup.py` data_files

Confirm `siyi_ros2/setup.py` includes these `data_files` entries (Phase 1 should
have them, but verify and add if missing):

```python
(os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
(os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
```

---

## Acceptance Criteria

- Both launch files and both YAML files exist with correct content
- `colcon build --packages-select siyi_ros2` installs launch files to `install/`
- `ros2 launch siyi_ros2 siyi.launch.py --show-args` shows all declared arguments
- `ros2 launch siyi_ros2 siyi_full.launch.py --show-args` shows all declared arguments
- No Python syntax errors in launch files
