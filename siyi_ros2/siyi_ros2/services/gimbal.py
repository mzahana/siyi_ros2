"""Gimbal control ROS2 services for SIYI cameras."""

from __future__ import annotations

from siyi_msgs.srv import CenterGimbal, SetGimbalAttitude, SetGimbalMode, SetGimbalSpeed
from siyi_sdk.models import GimbalMotionMode


class GimbalServices:
    """Registers and handles gimbal control services on the ROS2 node."""

    def __init__(self, node, bridge, client) -> None:
        self._node = node
        self._bridge = bridge
        self._client = client

        node.create_service(
            SetGimbalAttitude, "/siyi/set_attitude", self._handle_set_attitude
        )
        node.create_service(
            SetGimbalSpeed, "/siyi/set_speed", self._handle_set_speed
        )
        node.create_service(
            CenterGimbal, "/siyi/center", self._handle_center
        )
        node.create_service(
            SetGimbalMode, "/siyi/set_mode", self._handle_set_mode
        )

    def _handle_set_attitude(
        self,
        request: SetGimbalAttitude.Request,
        response: SetGimbalAttitude.Response,
    ) -> SetGimbalAttitude.Response:
        try:
            result = self._bridge.run_async(
                self._client.set_attitude(
                    yaw_deg=request.yaw_deg, pitch_deg=request.pitch_deg
                ),
                timeout=5.0,
            )
            response.success = True
            response.message = "OK"
            response.actual_yaw_deg = float(result.yaw_deg)
            response.actual_pitch_deg = float(result.pitch_deg)
            response.actual_roll_deg = float(result.roll_deg)
        except TimeoutError:
            response.success = False
            response.message = "Timeout waiting for SIYI response"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            self._node.get_logger().error(f"set_attitude service call failed: {exc}")
        return response

    def _handle_set_speed(
        self,
        request: SetGimbalSpeed.Request,
        response: SetGimbalSpeed.Response,
    ) -> SetGimbalSpeed.Response:
        try:
            self._bridge.run_async(
                self._client.rotate(yaw=request.yaw_speed, pitch=request.pitch_speed),
                timeout=5.0,
            )
            response.success = True
            response.message = "OK"
        except TimeoutError:
            response.success = False
            response.message = "Timeout waiting for SIYI response"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            self._node.get_logger().error(f"set_speed service call failed: {exc}")
        return response

    def _handle_center(
        self,
        request: CenterGimbal.Request,
        response: CenterGimbal.Response,
    ) -> CenterGimbal.Response:
        try:
            self._bridge.run_async(
                self._client.one_key_centering(),
                timeout=5.0,
            )
            response.success = True
            response.message = "OK"
        except TimeoutError:
            response.success = False
            response.message = "Timeout waiting for SIYI response"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            self._node.get_logger().error(f"center service call failed: {exc}")
        return response

    def _handle_set_mode(
        self,
        request: SetGimbalMode.Request,
        response: SetGimbalMode.Response,
    ) -> SetGimbalMode.Response:
        # The siyi_sdk does not expose a set_gimbal_mode command — only
        # get_gimbal_mode is available. Mode changes are done via
        # CaptureFuncType (LOCK_MODE, FOLLOW_MODE, FPV_MODE) through
        # client.capture(), but that is fire-and-forget with no ACK.
        # We therefore use capture() and report success if no exception.
        from siyi_sdk.models import CaptureFuncType

        _MODE_TO_CAPTURE: dict[int, CaptureFuncType] = {
            0: CaptureFuncType.LOCK_MODE,
            1: CaptureFuncType.FOLLOW_MODE,
            2: CaptureFuncType.FPV_MODE,
        }

        mode_int = int(request.mode)
        capture_func = _MODE_TO_CAPTURE.get(mode_int)

        if capture_func is None:
            response.success = False
            response.message = (
                f"Unknown mode {mode_int}. Expected 0=lock, 1=follow, 2=fpv."
            )
            return response

        try:
            self._bridge.run_async(
                self._client.capture(capture_func),
                timeout=5.0,
            )
            response.success = True
            response.message = (
                f"Mode change command sent (mode={GimbalMotionMode(mode_int).name}). "
                "Note: fire-and-forget; verify with /siyi/get_gimbal_info."
            )
        except TimeoutError:
            response.success = False
            response.message = "Timeout waiting for SIYI response"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            self._node.get_logger().error(f"set_mode service call failed: {exc}")
        return response
