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
