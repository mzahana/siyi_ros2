"""SIYI camera ROS2 node — streams RTSP video as sensor_msgs/Image topics."""

from __future__ import annotations

import asyncio
import threading
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, CompressedImage

from siyi_sdk.stream import SIYIStream, build_rtsp_url
from siyi_sdk.stream.models import (
    StreamConfig,
    StreamBackend,
    CAMERA_GENERATION_MAP,
    CameraGeneration,
    StreamFrame,
)

# Lazy import cv_bridge so that import errors surface at runtime with a clear message.
try:
    from cv_bridge import CvBridge
    _CV_BRIDGE_AVAILABLE = True
except Exception:
    _CV_BRIDGE_AVAILABLE = False


class SIYICameraNode(Node):
    """Streams RTSP video from a SIYI camera and publishes ROS2 image topics.

    Published topics:
        /siyi/image_raw           (sensor_msgs/Image)
        /siyi/camera_info         (sensor_msgs/CameraInfo)
        /siyi/image_compressed    (sensor_msgs/CompressedImage, optional)

    Parameters:
        rtsp_url        (str)  Override RTSP URL; auto-built when empty.
        camera_model    (str)  One of: zt30, zt6, zr30, zr10, a8, a2, r1m.
        host            (str)  Camera IP address.
        stream_index    (int)  0 = main stream, 1 = sub stream.
        backend         (str)  gstreamer / opencv / aiortsp / auto.
        image_encoding  (str)  ROS2 image encoding (e.g. bgr8, rgb8).
        publish_compressed (bool) Publish CompressedImage topic.
        frame_id        (str)  TF frame ID stamped on all messages.
    """

    def __init__(self) -> None:
        super().__init__("siyi_camera_node")

        self._bridge_cv: Optional[CvBridge] = None
        if _CV_BRIDGE_AVAILABLE:
            self._bridge_cv = CvBridge()
        else:
            self.get_logger().warning(
                "cv_bridge not available — Image messages will be built manually."
            )

        self._stream: Optional[SIYIStream] = None
        self._unsub = None

        # Dedicated asyncio event loop for SIYIStream (runs in its own thread).
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(
            target=self._run_loop, name="siyi_stream_asyncio", daemon=True
        )
        self._thread.start()

        self._declare_parameters()
        self._setup_publishers()
        self._start_stream()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _run_loop(self) -> None:
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    def _declare_parameters(self) -> None:
        self.declare_parameter("rtsp_url", "")
        self.declare_parameter("camera_model", "zt30")
        self.declare_parameter("host", "192.168.144.25")
        self.declare_parameter("stream_index", 0)
        self.declare_parameter("backend", "gstreamer")
        self.declare_parameter("image_encoding", "bgr8")
        self.declare_parameter("publish_compressed", True)
        self.declare_parameter("frame_id", "siyi_camera")

    def _build_rtsp_url(self) -> str:
        """Return RTSP URL from parameter or auto-construct from model/host/stream_index."""
        rtsp_url = self.get_parameter("rtsp_url").value
        if rtsp_url:
            return rtsp_url

        host = self.get_parameter("host").value
        camera_model = self.get_parameter("camera_model").value.lower()
        stream_index = self.get_parameter("stream_index").value

        gen = CAMERA_GENERATION_MAP.get(camera_model, CameraGeneration.NEW)
        stream_slot = "main" if stream_index == 0 else "sub"
        return build_rtsp_url(host=host, stream=stream_slot, generation=gen)

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

        backend_map: dict[str, StreamBackend] = {
            "gstreamer": StreamBackend.GSTREAMER,
            "opencv": StreamBackend.OPENCV,
            "aiortsp": StreamBackend.AIORTSP,
            "auto": StreamBackend.AUTO,
        }
        backend = backend_map.get(backend_str, StreamBackend.GSTREAMER)

        config = StreamConfig(rtsp_url=url, backend=backend)
        self._stream = SIYIStream(config)
        self._unsub = self._stream.on_frame(self._on_frame)

        asyncio.run_coroutine_threadsafe(self._stream.start(), self._loop)
        self.get_logger().info(
            f"SIYI camera stream started: {url} [backend={backend_str}]"
        )

    # ------------------------------------------------------------------
    # Frame callback — called from asyncio thread via SIYIStream dispatch
    # ------------------------------------------------------------------

    def _on_frame(self, frame: StreamFrame) -> None:
        """Handle a decoded video frame and publish ROS2 messages."""
        img_array: np.ndarray = frame.frame
        if img_array is None or img_array.size == 0:
            return

        stamp = self.get_clock().now().to_msg()
        frame_id: str = self.get_parameter("frame_id").value
        encoding: str = self.get_parameter("image_encoding").value

        # --- sensor_msgs/Image ---
        try:
            if self._bridge_cv is not None:
                img_msg = self._bridge_cv.cv2_to_imgmsg(img_array, encoding=encoding)
            else:
                img_msg = self._numpy_to_imgmsg(img_array, encoding)
            img_msg.header.stamp = stamp
            img_msg.header.frame_id = frame_id
            self._image_pub.publish(img_msg)
        except Exception as exc:
            self.get_logger().warning(
                f"Image publish error: {exc}", throttle_duration_sec=5.0
            )

        # --- sensor_msgs/CameraInfo (minimal, no calibration) ---
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = frame_id
        info.width = img_array.shape[1]
        info.height = img_array.shape[0]
        self._info_pub.publish(info)

        # --- sensor_msgs/CompressedImage ---
        if self._compressed_pub is not None:
            try:
                ok, buf = cv2.imencode(
                    ".jpg", img_array, [cv2.IMWRITE_JPEG_QUALITY, 85]
                )
                if ok:
                    comp = CompressedImage()
                    comp.header.stamp = stamp
                    comp.header.frame_id = frame_id
                    comp.format = "jpeg"
                    comp.data = buf.tobytes()
                    self._compressed_pub.publish(comp)
            except Exception as exc:
                self.get_logger().warning(
                    f"Compressed publish error: {exc}", throttle_duration_sec=5.0
                )

    # ------------------------------------------------------------------
    # Fallback Image builder (no cv_bridge)
    # ------------------------------------------------------------------

    @staticmethod
    def _numpy_to_imgmsg(arr: np.ndarray, encoding: str) -> Image:
        """Convert a numpy BGR array to sensor_msgs/Image without cv_bridge."""
        msg = Image()
        msg.height = arr.shape[0]
        msg.width = arr.shape[1]
        msg.encoding = encoding
        msg.is_bigendian = False
        msg.step = arr.strides[0]
        msg.data = arr.tobytes()
        return msg

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def destroy_node(self) -> None:
        if self._unsub is not None:
            self._unsub()

        if self._stream is not None:
            future = asyncio.run_coroutine_threadsafe(
                self._stream.stop(), self._loop
            )
            try:
                future.result(timeout=5.0)
            except Exception as exc:
                self.get_logger().warning(f"Stream stop error: {exc}")

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
