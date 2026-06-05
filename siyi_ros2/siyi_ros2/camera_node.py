"""SIYI camera ROS2 node — streams RTSP video as sensor_msgs/Image topics."""

from __future__ import annotations

import asyncio
import threading
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CameraInfo, CompressedImage

from siyi_sdk import configure_logging
from siyi_sdk.stream import SIYIStream, build_rtsp_url
from siyi_sdk.stream.models import (
    StreamConfig,
    StreamBackend,
    CAMERA_GENERATION_MAP,
    CameraGeneration,
    StreamFrame,
)

try:
    from cv_bridge import CvBridge
    _CV_BRIDGE_AVAILABLE = True
except Exception:
    _CV_BRIDGE_AVAILABLE = False

try:
    from camera_info_manager import CameraInfoManager
    _CAMERA_INFO_MANAGER_AVAILABLE = True
except ImportError:
    _CAMERA_INFO_MANAGER_AVAILABLE = False

# BEST_EFFORT: publisher never blocks waiting for subscriber ACKs — critical for
# high-frequency large image messages where DDS backpressure kills throughput.
_IMAGE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


class SIYICameraNode(Node):
    """Streams RTSP video from a SIYI camera and publishes ROS2 image topics.

    Published topics:
        /siyi/image_raw           (sensor_msgs/Image)
        /siyi/camera_info         (sensor_msgs/CameraInfo)
        /siyi/image_compressed    (sensor_msgs/CompressedImage, optional)

    Parameters:
        rtsp_url        (str)   Override RTSP URL; auto-built when empty.
        camera_model    (str)   One of: zt30, zt6, zr30, zr10, a8, a2, r1m.
        host            (str)   Camera IP address.
        stream_index    (int)   0 = main stream, 1 = sub stream.
        backend         (str)   gstreamer / opencv / aiortsp / auto.
        image_encoding  (str)   ROS2 image encoding (e.g. bgr8, rgb8).
        publish_raw     (bool)  Publish raw sensor_msgs/Image topic.
        publish_compressed (bool) Publish CompressedImage topic.
        image_scale     (float) Scaling factor (0.1–1.0) applied before publish.
        jpeg_quality    (int)   JPEG quality for compressed topic (1–100).
        latency_ms      (int)   GStreamer rtspsrc latency buffer in ms (0 = lowest).
        frame_id        (str)   TF frame ID stamped on all messages.
        camera_name     (str)   Camera name used by camera_info_manager.
        camera_info_url (str)   URL to camera calibration YAML.
                                Supported schemes:
                                  file:///abs/path/to/calib.yaml
                                  package://pkg_name/config/calib.yaml
                                Leave empty to publish uncalibrated CameraInfo.
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

        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(
            target=self._run_loop, name="siyi_stream_asyncio", daemon=True
        )
        self._thread.start()

        self._declare_parameters()

        # Cache hot-path parameters to avoid per-frame ROS2 parameter lookups.
        self._encoding: str = self.get_parameter("image_encoding").value
        self._frame_id: str = self.get_parameter("frame_id").value
        self._scale: float = self.get_parameter("image_scale").value
        self._jpeg_quality: int = self.get_parameter("jpeg_quality").value

        self._camera_info: Optional[CameraInfo] = self._load_camera_info()

        # Raw/CameraInfo publish pipeline.
        self._latest_frame: Optional[tuple[np.ndarray, rclpy.time.Time]] = None
        self._raw_lock = threading.Lock()
        self._raw_event = threading.Event()
        self._stop_event = threading.Event()

        # Compressed publish pipeline — separate thread so JPEG encoding never
        # blocks the raw publish loop.
        self._comp_frame: Optional[tuple[np.ndarray, rclpy.time.Time]] = None
        self._comp_lock = threading.Lock()
        self._comp_event = threading.Event()

        self._setup_publishers()

        self._raw_thread = threading.Thread(
            target=self._raw_worker, name="siyi_raw_publisher", daemon=True
        )
        self._raw_thread.start()

        if self.get_parameter("publish_compressed").value:
            self._comp_thread: Optional[threading.Thread] = threading.Thread(
                target=self._comp_worker, name="siyi_comp_publisher", daemon=True
            )
            self._comp_thread.start()
        else:
            self._comp_thread = None

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
        self.declare_parameter("publish_raw", True)
        self.declare_parameter("publish_compressed", True)
        self.declare_parameter("image_scale", 1.0)
        self.declare_parameter("jpeg_quality", 80)
        self.declare_parameter("latency_ms", 0)
        self.declare_parameter("frame_id", "siyi_camera")
        self.declare_parameter("camera_name", "siyi_camera")
        self.declare_parameter("camera_info_url", "")

    def _load_camera_info(self) -> Optional[CameraInfo]:
        """Load calibration via camera_info_manager. Returns None when uncalibrated."""
        url: str = self.get_parameter("camera_info_url").value
        if not url:
            return None

        if not _CAMERA_INFO_MANAGER_AVAILABLE:
            self.get_logger().error(
                "camera_info_url is set but camera_info_manager is not installed. "
                "Install it with: sudo apt install ros-<distro>-camera-info-manager"
            )
            return None

        camera_name: str = self.get_parameter("camera_name").value
        try:
            manager = CameraInfoManager(self, cname=camera_name, url=url)
            manager.loadCameraInfo()
            if not manager.isCalibrated():
                self.get_logger().warning(
                    f"camera_info_manager loaded '{url}' but reported uncalibrated. "
                    "Check that the file exists and is a valid ROS2 calibration YAML."
                )
                return None
            info = manager.getCameraInfo()
            self.get_logger().info(
                f"Camera calibration loaded from '{url}' "
                f"({info.width}x{info.height}, model={info.distortion_model})"
            )
            return info
        except Exception as exc:
            self.get_logger().error(f"Failed to load camera calibration from '{url}': {exc}")
            return None

    def _build_rtsp_url(self) -> str:
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
        if self.get_parameter("publish_raw").value:
            self._image_pub = self.create_publisher(Image, "/siyi/image_raw", _IMAGE_QOS)
        else:
            self._image_pub = None

        self._info_pub = self.create_publisher(CameraInfo, "/siyi/camera_info", _IMAGE_QOS)

        if self.get_parameter("publish_compressed").value:
            self._compressed_pub = self.create_publisher(
                CompressedImage, "/siyi/image_compressed", _IMAGE_QOS
            )
        else:
            self._compressed_pub = None

    def _start_stream(self) -> None:
        url = self._build_rtsp_url()
        backend_str = self.get_parameter("backend").value
        latency_ms = self.get_parameter("latency_ms").value

        backend_map: dict[str, StreamBackend] = {
            "gstreamer": StreamBackend.GSTREAMER,
            "opencv": StreamBackend.OPENCV,
            "aiortsp": StreamBackend.AIORTSP,
            "auto": StreamBackend.AUTO,
        }
        backend = backend_map.get(backend_str, StreamBackend.GSTREAMER)

        config = StreamConfig(rtsp_url=url, backend=backend, latency_ms=latency_ms)
        self._stream = SIYIStream(config)
        self._unsub = self._stream.on_frame(self._on_frame)

        asyncio.run_coroutine_threadsafe(self._stream.start(), self._loop)
        self.get_logger().info(
            f"SIYI camera stream started: {url} [backend={backend_str}, latency_ms={latency_ms}]"
        )

    # ------------------------------------------------------------------
    # Frame callback — called from asyncio thread; must be fast/non-blocking
    # ------------------------------------------------------------------

    def _on_frame(self, frame: StreamFrame) -> None:
        """Buffer the raw frame for the publish workers. No CPU work here."""
        img: np.ndarray = frame.frame
        if img is None or img.size == 0:
            return

        stamp = self.get_clock().now()

        with self._raw_lock:
            self._latest_frame = (img, stamp)
            self._raw_event.set()

    # ------------------------------------------------------------------
    # Raw + CameraInfo publisher worker
    # ------------------------------------------------------------------

    def _raw_worker(self) -> None:
        """Publishes raw Image and CameraInfo. Resize happens here, off asyncio."""
        while rclpy.ok() and not self._stop_event.is_set():
            if not self._raw_event.wait(timeout=0.1):
                continue
            self._raw_event.clear()

            with self._raw_lock:
                if self._latest_frame is None:
                    continue
                img, stamp = self._latest_frame
                self._latest_frame = None

            # Resize off the asyncio thread.
            scale = self._scale
            if scale != 1.0:
                w = int(img.shape[1] * scale)
                h = int(img.shape[0] * scale)
                img = cv2.resize(img, (w, h), interpolation=cv2.INTER_LINEAR)

            # Dispatch compressed encoding to its own thread before publishing raw
            # so both proceed in parallel.
            if self._compressed_pub is not None:
                with self._comp_lock:
                    self._comp_frame = (img, stamp)
                    self._comp_event.set()

            # --- sensor_msgs/Image ---
            if self._image_pub is not None:
                try:
                    encoding = self._encoding
                    if self._bridge_cv is not None:
                        img_msg = self._bridge_cv.cv2_to_imgmsg(img, encoding=encoding)
                    else:
                        img_msg = self._numpy_to_imgmsg(img, encoding)
                    img_msg.header.stamp = stamp.to_msg()
                    img_msg.header.frame_id = self._frame_id
                    self._image_pub.publish(img_msg)
                except Exception as exc:
                    self.get_logger().warning(
                        f"Image publish error: {exc}", throttle_duration_sec=5.0
                    )

            # --- sensor_msgs/CameraInfo ---
            if self._camera_info is not None:
                info = self._camera_info
            else:
                info = CameraInfo()
                info.width = img.shape[1]
                info.height = img.shape[0]
            info.header.stamp = stamp.to_msg()
            info.header.frame_id = self._frame_id
            self._info_pub.publish(info)

    # ------------------------------------------------------------------
    # Compressed publisher worker — JPEG encoding never blocks raw publish
    # ------------------------------------------------------------------

    def _comp_worker(self) -> None:
        """Encodes and publishes CompressedImage independently of raw publish."""
        encode_params = [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality]
        while rclpy.ok() and not self._stop_event.is_set():
            if not self._comp_event.wait(timeout=0.1):
                continue
            self._comp_event.clear()

            with self._comp_lock:
                if self._comp_frame is None:
                    continue
                img, stamp = self._comp_frame
                self._comp_frame = None

            if self._compressed_pub is None:
                continue

            try:
                ok, buf = cv2.imencode(".jpg", img, encode_params)
                if ok:
                    comp = CompressedImage()
                    comp.header.stamp = stamp.to_msg()
                    comp.header.frame_id = self._frame_id
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

        self._stop_event.set()
        self._raw_event.set()
        self._comp_event.set()
        self._raw_thread.join(timeout=2.0)
        if self._comp_thread is not None:
            self._comp_thread.join(timeout=2.0)
        self._thread.join(timeout=5.0)
        super().destroy_node()


def main(args=None) -> None:
    configure_logging()
    rclpy.init(args=args)
    node = SIYICameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
