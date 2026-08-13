"""
Launch file for the SIYI A8 mini TF description.

Spawns:
  • robot_state_publisher  – broadcasts the static TF tree from the URDF
                             (gimbal_base → camera_link → camera_optical_link)
  • static_transform_publisher – broadcasts the static TF that attaches
                                  gimbal_base to the vehicle body frame

Usage examples
──────────────
  # Default: attach to 'base_link'
  ros2 launch siyi_ros2 siyi_a8_tf.launch.py

  # Custom parent frame and offset (e.g. gimbal mounted 0.1 m in front of base)
  ros2 launch siyi_ros2 siyi_a8_tf.launch.py \\
      parent_frame:=base_link \\
      x:=0.1 y:=0.0 z:=0.05 \\
      roll:=0.0 pitch:=0.0 yaw:=0.0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("siyi_ros2")

    # ── launch arguments ────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument(
            "parent_frame",
            default_value="base_link",
            description="Frame to which gimbal_base is attached (vehicle body frame)",
        ),
        DeclareLaunchArgument(
            "x", default_value="0.0",
            description="X offset of gimbal_base w.r.t. parent_frame (metres)",
        ),
        DeclareLaunchArgument(
            "y", default_value="0.0",
            description="Y offset of gimbal_base w.r.t. parent_frame (metres)",
        ),
        DeclareLaunchArgument(
            "z", default_value="0.0",
            description="Z offset of gimbal_base w.r.t. parent_frame (metres)",
        ),
        DeclareLaunchArgument(
            "roll",  default_value="0.0",
            description="Roll  of gimbal_base w.r.t. parent_frame (radians)",
        ),
        DeclareLaunchArgument(
            "pitch", default_value="0.0",
            description="Pitch of gimbal_base w.r.t. parent_frame (radians)",
        ),
        DeclareLaunchArgument(
            "yaw",   default_value="0.0",
            description="Yaw   of gimbal_base w.r.t. parent_frame (radians)",
        ),
        DeclareLaunchArgument(
            "urdf_file",
            default_value=PathJoinSubstitution(
                [pkg_share, "urdf", "a8_mini.urdf.xacro"]
            ),
            description="Absolute path to the A8 mini URDF/xacro file",
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Optional node namespace",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Launch RViz2 with pre-configured siyi_a8 display (true/false)",
        ),
    ]

    # ── process xacro → robot_description ───────────────────────────────────
    # Wrap in ParameterValue(value_type=str) so launch doesn't try to parse
    # the XML string as YAML (which would throw the 'unable to parse' error).
    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", LaunchConfiguration("urdf_file")]),
        value_type=str,
    )

    # ── robot_state_publisher ────────────────────────────────────────────────
    #   Publishes all *static* joints in the URDF as /tf_static transforms:
    #     gimbal_base → camera_link
    #     camera_link → camera_optical_link
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="siyi_a8_state_publisher",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                # publish_frequency: robot_state_publisher will forward
                # /joint_states → /tf at whatever rate JointState arrives.
                # AttitudePublisher drives this at attitude_stream_hz (default 50 Hz).
                "publish_frequency": 50.0,
                # Use the frame names as-is (no namespace prefix)
                "frame_prefix": "",
            }
        ],
    )

    # ── static_transform_publisher ───────────────────────────────────────────
    #   Attaches gimbal_base to the parent (vehicle) frame.
    static_tf_gimbal_to_parent = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="siyi_a8_static_tf",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        arguments=[
            LaunchConfiguration("x"),
            LaunchConfiguration("y"),
            LaunchConfiguration("z"),
            LaunchConfiguration("yaw"),
            LaunchConfiguration("pitch"),
            LaunchConfiguration("roll"),
            LaunchConfiguration("parent_frame"),
            "gimbal_base",
        ],
    )

    # ── gimbal_tf_bridge ─────────────────────────────────────────────────────
    #   Subscribes to /siyi/attitude (published by siyi_node) and republishes
    #   as /joint_states so robot_state_publisher can drive the three revolute
    #   joints (gimbal_yaw, gimbal_pitch, gimbal_roll) in the URDF dynamically.
    #
    #   Data flow:
    #     siyi_node  →  /siyi/attitude  →  gimbal_tf_bridge
    #                                              │
    #                                       /joint_states
    #                                              │
    #                                   robot_state_publisher
    #                                              │
    #                                            /tf  (dynamic joint TFs)
    gimbal_tf_bridge = Node(
        package="siyi_ros2",
        executable="gimbal_tf_bridge",
        name="gimbal_tf_bridge",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        emulate_tty=True,
    )

    # ── RViz2 (optional) ─────────────────────────────────────────────────────
    #   Pre-configured with: RobotModel (TRANSIENT_LOCAL QoS), TF display,
    #   fixed frame = gimbal_base.
    #   Launch with:  rviz:=true
    rviz_config = PathJoinSubstitution(
        [pkg_share, "config", "siyi_a8_rviz.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
    )

    return LaunchDescription(
        args + [robot_state_publisher, static_tf_gimbal_to_parent, gimbal_tf_bridge, rviz_node]
    )
