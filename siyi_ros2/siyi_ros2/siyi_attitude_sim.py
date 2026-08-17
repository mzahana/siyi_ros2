"""
siyi_attitude_sim  –  Simulated SIYI gimbal attitude publisher.

Publishes  siyi_msgs/GimbalAttitude  on  /siyi/attitude  at a fixed
rate with smoothly sweeping yaw / pitch / roll angles so you can test
the full TF pipeline (gimbal_tf_bridge → robot_state_publisher → /tf)
without physical hardware.

Parameters
──────────
  rate_hz       float   Publish rate  (default: 20.0)
  yaw_amp_deg   float   Yaw   sweep ±amplitude in degrees  (default: 90.0)
  pitch_amp_deg float   Pitch sweep ±amplitude in degrees  (default: 45.0)
  roll_amp_deg  float   Roll  sweep ±amplitude in degrees  (default: 20.0)
  yaw_period_s  float   Seconds per full yaw   sweep cycle (default: 10.0)
  pitch_period_s float  Seconds per full pitch sweep cycle (default:  7.0)
  roll_period_s  float  Seconds per full roll  sweep cycle (default:  5.0)

Usage
─────
  ros2 run siyi_ros2 siyi_attitude_sim

  # custom sweep
  ros2 run siyi_ros2 siyi_attitude_sim \\
      --ros-args -p yaw_amp_deg:=135.0 -p rate_hz:=50.0
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from siyi_msgs.msg import GimbalAttitude


class SIYIAttitudeSim(Node):
    """Publishes a slowly sweeping GimbalAttitude for hardware-free testing."""

    def __init__(self) -> None:
        super().__init__("siyi_attitude_sim")

        # ── parameters ────────────────────────────────────────────────────
        self.declare_parameter("rate_hz",        20.0)
        self.declare_parameter("yaw_amp_deg",    90.0)
        self.declare_parameter("pitch_amp_deg",  45.0)
        self.declare_parameter("roll_amp_deg",   20.0)
        self.declare_parameter("yaw_period_s",   10.0)
        self.declare_parameter("pitch_period_s",  7.0)
        self.declare_parameter("roll_period_s",   5.0)

        rate_hz         = self.get_parameter("rate_hz").value
        self._yaw_amp   = self.get_parameter("yaw_amp_deg").value
        self._pitch_amp = self.get_parameter("pitch_amp_deg").value
        self._roll_amp  = self.get_parameter("roll_amp_deg").value
        self._yaw_T     = self.get_parameter("yaw_period_s").value
        self._pitch_T   = self.get_parameter("pitch_period_s").value
        self._roll_T    = self.get_parameter("roll_period_s").value

        # ── publisher ─────────────────────────────────────────────────────
        self._pub = self.create_publisher(
            GimbalAttitude, "/siyi/attitude", qos_profile_sensor_data
        )

        # ── timer ─────────────────────────────────────────────────────────
        period = 1.0 / rate_hz
        self._timer = self.create_timer(period, self._publish)

        self.get_logger().info(
            f"siyi_attitude_sim running at {rate_hz:.1f} Hz\n"
            f"  yaw  : ±{self._yaw_amp}°  period={self._yaw_T}s\n"
            f"  pitch: ±{self._pitch_amp}°  period={self._pitch_T}s\n"
            f"  roll : ±{self._roll_amp}°  period={self._roll_T}s"
        )

    def _publish(self) -> None:
        now   = self.get_clock().now()
        stamp = now.to_msg()
        t     = now.nanoseconds * 1e-9          # seconds since epoch

        # Smooth sine-wave sweeps – each axis has its own period
        yaw_deg   = self._yaw_amp   * math.sin(2 * math.pi * t / self._yaw_T)
        pitch_deg = self._pitch_amp * math.sin(2 * math.pi * t / self._pitch_T)
        roll_deg  = self._roll_amp  * math.sin(2 * math.pi * t / self._roll_T)

        # Rates: derivative of the sine → amplitude × (2π/T) × cos
        yaw_rate   = self._yaw_amp   * (2*math.pi/self._yaw_T)   * math.cos(2*math.pi*t/self._yaw_T)
        pitch_rate = self._pitch_amp * (2*math.pi/self._pitch_T)  * math.cos(2*math.pi*t/self._pitch_T)
        roll_rate  = self._roll_amp  * (2*math.pi/self._roll_T)   * math.cos(2*math.pi*t/self._roll_T)

        msg = GimbalAttitude()
        msg.header.stamp    = stamp
        msg.header.frame_id = "gimbal_base"
        msg.yaw_deg         = yaw_deg
        msg.pitch_deg       = pitch_deg
        msg.roll_deg        = roll_deg
        msg.yaw_rate_dps    = yaw_rate
        msg.pitch_rate_dps  = pitch_rate
        msg.roll_rate_dps   = roll_rate
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SIYIAttitudeSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
