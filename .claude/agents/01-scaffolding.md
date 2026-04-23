---
name: 01-scaffolding
description: Phase 1 — creates the siyi_msgs and siyi_ros2 ROS2 package skeletons including package.xml, CMakeLists.txt, setup.py, setup.cfg, and empty module directories
model: claude-haiku-4-5-20251001
color: "#4A90D9"
---

# Phase 1 — Package Scaffolding

## Context

You are implementing Phase 1 of the `siyi_ros2` project.
Working directory: `~/ros2_ws/src/siyi_ros2/`
ROS2 version: Jazzy (Ubuntu 24.04)

This phase creates the complete package skeletons for two ROS2 packages:
- `siyi_msgs/` — ament_cmake package for custom message/service definitions
- `siyi_ros2/` — ament_python package for Python ROS2 nodes

## Tasks

### 1. Create `siyi_msgs/package.xml`

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>siyi_msgs</name>
  <version>0.1.0</version>
  <description>Custom message and service definitions for SIYI gimbal-camera systems</description>
  <maintainer email="mohamedashraf123@gmail.com">Mohamed Abdelkader</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>

  <exec_depend>rosidl_default_runtime</exec_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### 2. Create `siyi_msgs/CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.8)
project(siyi_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(msg_files
  "msg/GimbalAttitude.msg"
  "msg/LaserDistance.msg"
  "msg/GimbalInfo.msg"
  "msg/ZoomInfo.msg"
  "msg/AITrackingTarget.msg"
  "msg/FirmwareVersion.msg"
)

set(srv_files
  "srv/SetGimbalAttitude.srv"
  "srv/SetGimbalSpeed.srv"
  "srv/CenterGimbal.srv"
  "srv/SetAbsoluteZoom.srv"
  "srv/ManualZoom.srv"
  "srv/AutoFocus.srv"
  "srv/ManualFocus.srv"
  "srv/TakePhoto.srv"
  "srv/StartRecording.srv"
  "srv/StopRecording.srv"
  "srv/GetFirmwareVersion.srv"
  "srv/GetGimbalInfo.srv"
  "srv/SetOSD.srv"
  "srv/SetLaserRanging.srv"
  "srv/GetLaserDistance.srv"
  "srv/SetPseudoColor.srv"
  "srv/SetGimbalMode.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
```

### 3. Create `siyi_msgs/msg/` and `siyi_msgs/srv/` directories (empty placeholders)

Create these empty directories — Phase 2 fills the `.msg` and `.srv` files:
- `siyi_msgs/msg/.gitkeep`
- `siyi_msgs/srv/.gitkeep`

### 4. Create `siyi_ros2/package.xml`

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>siyi_ros2</name>
  <version>0.1.0</version>
  <description>ROS2 Jazzy interface for SIYI gimbal-camera systems</description>
  <maintainer email="mohamedashraf123@gmail.com">Mohamed Abdelkader</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>siyi_msgs</depend>

  <buildtool_depend>ament_python</buildtool_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### 5. Create `siyi_ros2/setup.py`

```python
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'siyi_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mohamed Abdelkader',
    maintainer_email='mohamedashraf123@gmail.com',
    description='ROS2 Jazzy interface for SIYI gimbal-camera systems',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'siyi_node = siyi_ros2.siyi_node:main',
            'siyi_camera_node = siyi_ros2.camera_node:main',
        ],
    },
)
```

### 6. Create `siyi_ros2/setup.cfg`

```ini
[develop]
script_dir=$base/lib/siyi_ros2
[install]
install_scripts=$base/lib/siyi_ros2
```

### 7. Create `siyi_ros2/resource/siyi_ros2`

Empty file (ROS2 package marker):
```
```

### 8. Create Python module directories and stubs

Create these files with minimal stub content:

**`siyi_ros2/siyi_ros2/__init__.py`** — empty

**`siyi_ros2/siyi_ros2/services/__init__.py`** — empty

**`siyi_ros2/siyi_ros2/publishers/__init__.py`** — empty

**`siyi_ros2/siyi_ros2/_async_bridge.py`** — stub only:
```python
"""Asyncio ↔ ROS2 thread bridge. Implemented in Phase 3."""
```

**`siyi_ros2/siyi_ros2/siyi_node.py`** — stub only:
```python
"""SIYI gimbal/camera ROS2 node. Implemented in Phase 3."""

def main():
    pass
```

**`siyi_ros2/siyi_ros2/camera_node.py`** — stub only:
```python
"""SIYI video streaming ROS2 node. Implemented in Phase 6."""

def main():
    pass
```

**`siyi_ros2/siyi_ros2/services/gimbal.py`** — stub only:
```python
"""Gimbal control services. Implemented in Phase 4."""
```

**`siyi_ros2/siyi_ros2/services/camera.py`** — stub only:
```python
"""Camera control services. Implemented in Phase 4."""
```

**`siyi_ros2/siyi_ros2/services/system.py`** — stub only:
```python
"""System info services. Implemented in Phase 4."""
```

**`siyi_ros2/siyi_ros2/publishers/attitude.py`** — stub only:
```python
"""Attitude publisher. Implemented in Phase 5."""
```

**`siyi_ros2/siyi_ros2/publishers/laser.py`** — stub only:
```python
"""Laser distance publisher. Implemented in Phase 5."""
```

**`siyi_ros2/siyi_ros2/publishers/ai_tracking.py`** — stub only:
```python
"""AI tracking publisher. Implemented in Phase 5."""
```

### 9. Create empty directories

Create the following directories (with `.gitkeep` files):
- `siyi_ros2/launch/`
- `siyi_ros2/config/`
- `siyi_ros2/test/`

## Acceptance Criteria

- `colcon build --packages-select siyi_msgs siyi_ros2` succeeds (stubs will build)
- All directories and files listed above exist
- No syntax errors in any Python file
- `setup.py` entry points reference correct module paths
