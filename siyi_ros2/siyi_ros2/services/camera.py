"""Camera control ROS2 services for SIYI cameras."""

from __future__ import annotations

from siyi_msgs.srv import (
    AutoFocus,
    ManualFocus,
    ManualZoom,
    SetAbsoluteZoom,
    SetOSD,
    SetPseudoColor,
    StartRecording,
    StopRecording,
    TakePhoto,
)
from siyi_sdk.models import CaptureFuncType, PseudoColor


class CameraServices:
    """Registers and handles camera control services on the ROS2 node."""

    def __init__(self, node, bridge, client) -> None:
        self._node = node
        self._bridge = bridge
        self._client = client

        node.create_service(TakePhoto, "/siyi/take_photo", self._handle_take_photo)
        node.create_service(
            StartRecording, "/siyi/start_recording", self._handle_start_recording
        )
        node.create_service(
            StopRecording, "/siyi/stop_recording", self._handle_stop_recording
        )
        node.create_service(
            SetAbsoluteZoom, "/siyi/set_zoom", self._handle_set_zoom
        )
        node.create_service(ManualZoom, "/siyi/manual_zoom", self._handle_manual_zoom)
        node.create_service(AutoFocus, "/siyi/auto_focus", self._handle_auto_focus)
        node.create_service(
            ManualFocus, "/siyi/manual_focus", self._handle_manual_focus
        )
        node.create_service(SetOSD, "/siyi/set_osd", self._handle_set_osd)
        node.create_service(
            SetPseudoColor, "/siyi/set_pseudo_color", self._handle_set_pseudo_color
        )

    def _handle_take_photo(
        self,
        request: TakePhoto.Request,
        response: TakePhoto.Response,
    ) -> TakePhoto.Response:
        try:
            self._bridge.run_async(
                self._client.capture(CaptureFuncType.PHOTO),
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
            self._node.get_logger().error(f"take_photo service call failed: {exc}")
        return response

    def _handle_start_recording(
        self,
        request: StartRecording.Request,
        response: StartRecording.Response,
    ) -> StartRecording.Response:
        try:
            self._bridge.run_async(
                self._client.capture(CaptureFuncType.START_RECORD),
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
            self._node.get_logger().error(f"start_recording service call failed: {exc}")
        return response

    def _handle_stop_recording(
        self,
        request: StopRecording.Request,
        response: StopRecording.Response,
    ) -> StopRecording.Response:
        # The SIYI SDK toggles recording on/off with the same START_RECORD command.
        try:
            self._bridge.run_async(
                self._client.capture(CaptureFuncType.START_RECORD),
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
            self._node.get_logger().error(f"stop_recording service call failed: {exc}")
        return response

    def _handle_set_zoom(
        self,
        request: SetAbsoluteZoom.Request,
        response: SetAbsoluteZoom.Response,
    ) -> SetAbsoluteZoom.Response:
        try:
            self._bridge.run_async(
                self._client.absolute_zoom(request.zoom_level),
                timeout=5.0,
            )
            actual = self._bridge.run_async(
                self._client.get_current_zoom(),
                timeout=5.0,
            )
            response.success = True
            response.message = "OK"
            response.actual_zoom = float(actual)
        except TimeoutError:
            response.success = False
            response.message = "Timeout waiting for SIYI response"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            self._node.get_logger().error(f"set_zoom service call failed: {exc}")
        return response

    def _handle_manual_zoom(
        self,
        request: ManualZoom.Request,
        response: ManualZoom.Response,
    ) -> ManualZoom.Response:
        try:
            current_zoom = self._bridge.run_async(
                self._client.manual_zoom(request.direction),
                timeout=5.0,
            )
            response.success = True
            response.message = "OK"
            response.current_zoom = float(current_zoom)
        except TimeoutError:
            response.success = False
            response.message = "Timeout waiting for SIYI response"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            self._node.get_logger().error(f"manual_zoom service call failed: {exc}")
        return response

    def _handle_auto_focus(
        self,
        request: AutoFocus.Request,
        response: AutoFocus.Response,
    ) -> AutoFocus.Response:
        try:
            self._bridge.run_async(
                self._client.auto_focus(
                    touch_x=int(request.touch_x), touch_y=int(request.touch_y)
                ),
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
            self._node.get_logger().error(f"auto_focus service call failed: {exc}")
        return response

    def _handle_manual_focus(
        self,
        request: ManualFocus.Request,
        response: ManualFocus.Response,
    ) -> ManualFocus.Response:
        try:
            self._bridge.run_async(
                self._client.manual_focus(request.direction),
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
            self._node.get_logger().error(f"manual_focus service call failed: {exc}")
        return response

    def _handle_set_osd(
        self,
        request: SetOSD.Request,
        response: SetOSD.Response,
    ) -> SetOSD.Response:
        try:
            result = self._bridge.run_async(
                self._client.set_osd_flag(request.enabled),
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
            self._node.get_logger().error(f"set_osd service call failed: {exc}")
        return response

    def _handle_set_pseudo_color(
        self,
        request: SetPseudoColor.Request,
        response: SetPseudoColor.Response,
    ) -> SetPseudoColor.Response:
        try:
            color = PseudoColor(int(request.color_mode))
        except ValueError:
            response.success = False
            response.message = (
                f"Unknown color_mode {request.color_mode}. "
                f"Valid range: 0-{max(e.value for e in PseudoColor)}."
            )
            return response

        try:
            result = self._bridge.run_async(
                self._client.set_pseudo_color(color),
                timeout=5.0,
            )
            response.success = True
            response.message = "OK"
            response.actual_color_mode = int(result.value)
        except TimeoutError:
            response.success = False
            response.message = "Timeout waiting for SIYI response"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            self._node.get_logger().error(
                f"set_pseudo_color service call failed: {exc}"
            )
        return response
