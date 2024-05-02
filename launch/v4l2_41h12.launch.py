from pathlib import Path
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    device_arg = DeclareLaunchArgument(
        name="device",
        default_value="/dev/video0",
    )

    config_path = Path(get_package_share_directory("apriltag_ros")) / "cfg" / "tags_41h12.yaml"
    with open(config_path, "r") as file:
        config = yaml.safe_load(file)["apriltag/apriltag"]["ros__parameters"]

    container = ComposableNodeContainer(
        package="rclcpp_components",
        executable="component_container",
        name="apriltag_container",
        namespace="",
        output="screen",
        composable_node_descriptions=[
            # camera node
            ComposableNode(
                package="v4l2_camera",
                plugin="v4l2_camera::V4L2Camera",
                name="v4l2",
                parameters=[{"video_device": LaunchConfiguration("device")}],
                extra_arguments=[{"use_intra_process_comms": True}]
            ),
            # # image proc node
            ComposableNode(
                package="image_proc",
                plugin="image_proc::RectifyNode",
                name="rectify",
                namespace="v4l2",
                remappings=[("image", "image_raw")],
                extra_arguments=[{"use_intra_process_comms": True}]
            ),
            # apriltag node
            ComposableNode(
                package="apriltag_ros",
                plugin="AprilTagNode",
                name="apriltag",
                namespace="apriltag",
                remappings=[
                    ("/apriltag/image_rect", "/v4l2/image_rect"),
                    ("/apriltag/camera_info", "/v4l2/camera_info"),
                ],
                # parameters=[{"family": "Standard41h12"}, {"size": 0.063}],
                parameters=[config],
                extra_arguments=[{"use_intra_process_comms": True}],
            )
        ],
    )

    return LaunchDescription([
        device_arg,
        container,
    ])
