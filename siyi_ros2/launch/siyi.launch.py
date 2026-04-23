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
