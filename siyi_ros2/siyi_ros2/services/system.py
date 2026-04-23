"""System information ROS2 services for SIYI cameras."""

from __future__ import annotations

from builtin_interfaces.msg import Time
from siyi_msgs.msg import GimbalInfo
from siyi_msgs.srv import GetFirmwareVersion, GetGimbalInfo, GetLaserDistance, SetLaserRanging
from siyi_sdk.models import FirmwareVersion, MountingDirection, RecordingState


class SystemServices:
    """Registers and handles system information services on the ROS2 node."""

    def __init__(self, node, bridge, client) -> None:
        self._node = node
        self._bridge = bridge
        self._client = client

        node.create_service(
            GetFirmwareVersion,
            "/siyi/get_firmware_version",
            self._handle_get_firmware_version,
        )
        node.create_service(
            GetGimbalInfo, "/siyi/get_gimbal_info", self._handle_get_gimbal_info
        )
        node.create_service(
            SetLaserRanging,
            "/siyi/set_laser_ranging",
            self._handle_set_laser_ranging,
        )
        node.create_service(
            GetLaserDistance,
            "/siyi/get_laser_distance",
            self._handle_get_laser_distance,
        )

    def _handle_get_firmware_version(
        self,
        request: GetFirmwareVersion.Request,
        response: GetFirmwareVersion.Response,
    ) -> GetFirmwareVersion.Response:
        try:
            fw = self._bridge.run_async(
                self._client.get_firmware_version(),
                timeout=5.0,
            )
            response.success = True
            response.message = "OK"
            response.camera_version = FirmwareVersion.format_word(fw.camera)
            response.gimbal_version = FirmwareVersion.format_word(fw.gimbal)
            response.zoom_version = FirmwareVersion.format_word(fw.zoom)
        except TimeoutError:
            response.success = False
            response.message = "Timeout waiting for SIYI response"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            self._node.get_logger().error(
                f"get_firmware_version service call failed: {exc}"
            )
        return response

    def _handle_get_gimbal_info(
        self,
        request: GetGimbalInfo.Request,
        response: GetGimbalInfo.Response,
    ) -> GetGimbalInfo.Response:
        try:
            info = self._bridge.run_async(
                self._client.get_camera_system_info(),
                timeout=5.0,
            )
            zoom_level = self._bridge.run_async(
                self._client.get_current_zoom(),
                timeout=5.0,
            )

            msg = GimbalInfo()
            now = self._node.get_clock().now().to_msg()
            msg.header.stamp = now
            msg.header.frame_id = "siyi_gimbal"

            # GimbalInfo.msg: motion_mode uint8 (0=lock, 1=follow, 2=fpv)
            msg.motion_mode = int(info.gimbal_motion_mode.value)

            # GimbalInfo.msg: mounting_dir uint8 (0=normal, 1=upside_down)
            # SDK MountingDirection: RESERVED=0, NORMAL=1, INVERTED=2
            # Map: NORMAL(1) -> 0 (normal), INVERTED(2) -> 1 (upside_down), RESERVED(0) -> 0
            if info.gimbal_mounting_dir == MountingDirection.INVERTED:
                msg.mounting_dir = 1
            else:
                msg.mounting_dir = 0

            # recording bool: True when RecordingState.RECORDING
            msg.recording = (info.record_sta == RecordingState.RECORDING)

            # hdr_enabled bool
            msg.hdr_enabled = bool(info.hdr_sta)

            # zoom_level float32
            msg.zoom_level = float(zoom_level)

            response.success = True
            response.message = "OK"
            response.info = msg
        except TimeoutError:
            response.success = False
            response.message = "Timeout waiting for SIYI response"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            self._node.get_logger().error(
                f"get_gimbal_info service call failed: {exc}"
            )
        return response

    def _handle_set_laser_ranging(
        self,
        request: SetLaserRanging.Request,
        response: SetLaserRanging.Response,
    ) -> SetLaserRanging.Response:
        try:
            result = self._bridge.run_async(
                self._client.set_laser_ranging_state(request.enabled),
                timeout=5.0,
            )
            response.success = bool(result)
            response.message = "OK" if result else "Device returned failure"
        except TimeoutError:
            response.success = False
            response.message = "Timeout waiting for SIYI response"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            self._node.get_logger().error(
                f"set_laser_ranging service call failed: {exc}"
            )
        return response

    def _handle_get_laser_distance(
        self,
        request: GetLaserDistance.Request,
        response: GetLaserDistance.Response,
    ) -> GetLaserDistance.Response:
        try:
            laser = self._bridge.run_async(
                self._client.get_laser_distance(),
                timeout=5.0,
            )
            response.success = True
            response.message = "OK"
            response.distance_m = float(laser.distance_m) if laser.distance_m is not None else 0.0
            response.valid = laser.distance_m is not None
        except TimeoutError:
            response.success = False
            response.message = "Timeout waiting for SIYI response"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            self._node.get_logger().error(
                f"get_laser_distance service call failed: {exc}"
            )
        return response
