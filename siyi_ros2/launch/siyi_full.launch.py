"""Launch siyi_node + siyi_camera_node (full stack: control + video streaming)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import SetEnvironmentVariable


def generate_launch_description():
    pkg_share = FindPackageShare("siyi_ros2")

    args = [
        DeclareLaunchArgument("host", default_value="192.168.144.25",
                              description="SIYI device IP address"),
        DeclareLaunchArgument("port", default_value="37260"),
        DeclareLaunchArgument("transport", default_value="udp"),
        DeclareLaunchArgument("attitude_stream_hz", default_value="10"),
        DeclareLaunchArgument("auto_reconnect", default_value="false"),
        DeclareLaunchArgument(
            "publish_tf", default_value="false",
            description="Broadcast dynamic TF from tf_parent_frame to tf_child_frame"),
        DeclareLaunchArgument(
            "tf_parent_frame", default_value="base_link",
            description="Parent TF frame (vehicle body frame)"),
        DeclareLaunchArgument(
            "tf_child_frame", default_value="siyi_gimbal",
            description="Child TF frame (gimbal frame)"),
        DeclareLaunchArgument("camera_model", default_value="zt30",
                              description="Camera model key for RTSP URL"),
        DeclareLaunchArgument("stream_index", default_value="0"),
        DeclareLaunchArgument("backend", default_value="gstreamer",
                              description="Video backend: gstreamer | opencv | aiortsp"),
        DeclareLaunchArgument("publish_raw", default_value="true"),
        DeclareLaunchArgument("publish_compressed", default_value="true"),
        DeclareLaunchArgument("image_scale", default_value="1.0"),
        DeclareLaunchArgument("jpeg_quality", default_value="80"),
        DeclareLaunchArgument("latency_ms", default_value="0"),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument(
            "camera_info_url", default_value="",
            description=(
                "URL to camera calibration YAML loaded via camera_info_manager. "
                "Examples: file:///abs/path/calib.yaml  "
                "or package://siyi_ros2/config/calib.yaml. "
                "Leave empty to publish uncalibrated CameraInfo."
            ),
        ),
        DeclareLaunchArgument(
            "camera_name", default_value="siyi_camera",
            description="Camera name used by camera_info_manager",
        ),
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
                "publish_tf": LaunchConfiguration("publish_tf"),
                "tf_parent_frame": LaunchConfiguration("tf_parent_frame"),
                "tf_child_frame": LaunchConfiguration("tf_child_frame"),
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
                "publish_raw": LaunchConfiguration("publish_raw"),
                "publish_compressed": LaunchConfiguration("publish_compressed"),
                "image_scale": LaunchConfiguration("image_scale"),
                "jpeg_quality": LaunchConfiguration("jpeg_quality"),
                "latency_ms": LaunchConfiguration("latency_ms"),
                "camera_info_url": LaunchConfiguration("camera_info_url"),
                "camera_name": LaunchConfiguration("camera_name"),
            },
        ],
        output="screen",
        emulate_tty=True,
        arguments=["--ros-args", "--log-level", "warn"],
    )

    suppress_gst_log = SetEnvironmentVariable("GST_DEBUG", "0")
    suppress_sdk_log = SetEnvironmentVariable("SIYI_LOG_LEVEL", "WARNING")
    suppress_trace = SetEnvironmentVariable("SIYI_PROTOCOL_TRACE", "0")

    return LaunchDescription(args + [suppress_gst_log, suppress_sdk_log, suppress_trace, siyi_node, camera_node])
