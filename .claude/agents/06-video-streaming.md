---
name: 06-video-streaming
description: Phase 6 — implements SIYICameraNode with RTSP video streaming via SIYIStream, publishing sensor_msgs/Image and sensor_msgs/CompressedImage topics
model: claude-sonnet-4-6
color: "#E74C3C"
---

# Phase 6 — Video Streaming Node

## Context

Working directory: `~/ros2_ws/src/siyi_ros2/`
siyi_sdk location: `~/src/siyi_sdk`
Phases 1–5 must be complete.

You are implementing `siyi_ros2/siyi_ros2/camera_node.py` — a standalone ROS2 node
that streams video from the SIYI camera via RTSP and publishes ROS2 image topics.

Read before writing:
- `~/src/siyi_sdk/siyi_sdk/stream/stream.py` — `SIYIStream` class
- `~/src/siyi_sdk/siyi_sdk/stream/models.py` — `StreamConfig`, `StreamFrame`, `StreamBackend`, `CameraGeneration`, `CAMERA_GENERATION_MAP`

---

## RTSP URL Construction

The siyi_sdk builds RTSP URLs based on camera generation (OLD vs NEW):
```python
from siyi_sdk.stream.models import StreamConfig, StreamBackend, CameraGeneration, CAMERA_GENERATION_MAP
```

If `rtsp_url` parameter is non-empty, use it directly.
Otherwise, construct via `StreamConfig` — read the `StreamConfig` dataclass to understand
what fields it takes (url is one of them). The RTSP URL format is:

- **NEW cameras** (zt30, zt6): `rtsp://{host}:8554/video{stream_index}`
- **OLD cameras** (zr30, zr10, a8, a2, r1m): `rtsp://{host}/`

Build the URL in the node based on `camera_model` and `host` parameters.

---

## Parameters

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `rtsp_url` | str | `""` | Override URL (auto-built if empty) |
| `camera_model` | str | `"zt30"` | `zt30`, `zt6`, `zr30`, `zr10`, `a8`, `a2`, `r1m` |
| `host` | str | `"192.168.144.25"` | Camera IP |
| `stream_index` | int | `0` | Stream index (0=main, 1=sub) |
| `backend` | str | `"gstreamer"` | `gstreamer` / `opencv` / `aiortsp` / `auto` |
| `image_encoding` | str | `"bgr8"` | ROS2 image encoding string |
| `publish_compressed` | bool | `True` | Publish CompressedImage topic |
| `frame_id` | str | `"siyi_camera"` | TF frame ID |

---

## Implementation: `camera_node.py`

```python
"""SIYI camera ROS2 node — streams RTSP video as sensor_msgs/Image topics."""

from __future__ import annotations

import asyncio
import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from cv_bridge import CvBridge

from siyi_sdk.stream import SIYIStream
from siyi_sdk.stream.models import StreamConfig, StreamBackend, CAMERA_GENERATION_MAP, CameraGeneration
```

### Class structure

```python
class SIYICameraNode(Node):

    def __init__(self) -> None:
        super().__init__("siyi_camera_node")
        self._bridge_cv = CvBridge()
        self._stream: SIYIStream | None = None
        self._unsub = None

        # asyncio loop for SIYIStream
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(
            target=self._run_loop, name="siyi_stream_asyncio", daemon=True
        )
        self._thread.start()

        self._declare_parameters()
        self._setup_publishers()
        self._start_stream()

    def _run_loop(self) -> None:
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    def _declare_parameters(self) -> None: ...

    def _build_rtsp_url(self) -> str:
        rtsp_url = self.get_parameter("rtsp_url").value
        if rtsp_url:
            return rtsp_url
        host = self.get_parameter("host").value
        camera_model = self.get_parameter("camera_model").value.lower()
        stream_index = self.get_parameter("stream_index").value
        gen = CAMERA_GENERATION_MAP.get(camera_model, CameraGeneration.NEW)
        if gen == CameraGeneration.NEW:
            return f"rtsp://{host}:8554/video{stream_index}"
        else:
            return f"rtsp://{host}/"

    def _setup_publishers(self) -> None:
        self._image_pub = self.create_publisher(Image, "/siyi/image_raw", 1)
        self._info_pub = self.create_publisher(CameraInfo, "/siyi/camera_info", 1)
        if self.get_parameter("publish_compressed").value:
            self._compressed_pub = self.create_publisher(
                CompressedImage, "/siyi/image_compressed", 1
            )
        else:
            self._compressed_pub = None

    def _start_stream(self) -> None:
        url = self._build_rtsp_url()
        backend_str = self.get_parameter("backend").value
        backend_map = {
            "gstreamer": StreamBackend.GSTREAMER,
            "opencv": StreamBackend.OPENCV,
            "aiortsp": StreamBackend.AIORTSP,
            "auto": StreamBackend.AUTO,
        }
        backend = backend_map.get(backend_str, StreamBackend.GSTREAMER)
        config = StreamConfig(url=url, backend=backend)
        self._stream = SIYIStream(config)
        self._unsub = self._stream.on_frame(self._on_frame)
        asyncio.run_coroutine_threadsafe(self._stream.start(), self._loop)
        self.get_logger().info(f"SIYI camera stream started: {url} [{backend_str}]")

    def _on_frame(self, frame) -> None:
        # frame is siyi_sdk StreamFrame — frame.frame is np.ndarray (BGR)
        img_array: np.ndarray = frame.frame
        if img_array is None:
            return

        stamp = self.get_clock().now().to_msg()
        frame_id = self.get_parameter("frame_id").value
        encoding = self.get_parameter("image_encoding").value

        # Publish Image
        try:
            img_msg = self._bridge_cv.cv2_to_imgmsg(img_array, encoding=encoding)
            img_msg.header.stamp = stamp
            img_msg.header.frame_id = frame_id
            self._image_pub.publish(img_msg)
        except Exception as exc:
            self.get_logger().warning(f"Image publish error: {exc}", throttle_duration_sec=5.0)

        # Publish CameraInfo (minimal — no calibration)
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = frame_id
        info.width = img_array.shape[1]
        info.height = img_array.shape[0]
        self._info_pub.publish(info)

        # Publish CompressedImage
        if self._compressed_pub is not None:
            try:
                ok, buf = cv2.imencode(".jpg", img_array, [cv2.IMWRITE_JPEG_QUALITY, 85])
                if ok:
                    comp = CompressedImage()
                    comp.header.stamp = stamp
                    comp.header.frame_id = frame_id
                    comp.format = "jpeg"
                    comp.data = buf.tobytes()
                    self._compressed_pub.publish(comp)
            except Exception as exc:
                self.get_logger().warning(f"Compressed publish error: {exc}", throttle_duration_sec=5.0)

    def destroy_node(self) -> None:
        if self._unsub is not None:
            self._unsub()
        if self._stream is not None:
            asyncio.run_coroutine_threadsafe(self._stream.stop(), self._loop).result(timeout=5.0)
        if self._loop.is_running():
            self._loop.call_soon_threadsafe(self._loop.stop)
        self._thread.join(timeout=5.0)
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SIYICameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

### Important implementation notes

1. **Read `StreamFrame`** from `~/src/siyi_sdk/siyi_sdk/stream/models.py` to confirm
   the field name for the numpy array (likely `frame.frame`).

2. **Read `StreamConfig`** fields carefully — construct it with `url=` and `backend=`.
   Check if `StreamConfig` takes only `url` and `backend`, or additional fields.

3. **cv_bridge** — `cv_bridge` must be available. Add `python3-cv-bridge` to `package.xml`
   as `<depend>python3-cv-bridge</depend>`.

4. **QoS for images** — use depth=1 (latest frame only, drop old ones) for low-latency streaming.

5. **GStreamer backend** — gives ~120 ms latency. Log a warning if GStreamer is not
   available and falls back to OpenCV.

---

## Update `siyi_ros2/package.xml`

Add these dependencies:
```xml
<depend>python3-opencv</depend>
<depend>python3-cv-bridge</depend>
<depend>sensor_msgs</depend>
```

---

## Acceptance Criteria

- `camera_node.py` implements `SIYICameraNode` with all parameters
- RTSP URL auto-construction works for both OLD and NEW camera generations
- `_on_frame` callback publishes `Image`, `CameraInfo`, and optionally `CompressedImage`
- `destroy_node` cleanly stops the stream and asyncio loop
- `package.xml` includes `python3-cv-bridge`
- `colcon build --packages-select siyi_ros2` succeeds
- No import errors when running: `python3 -c "from siyi_ros2.camera_node import SIYICameraNode"`
