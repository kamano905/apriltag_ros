from pathlib import Path
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    config_path = Path(get_package_share_directory("apriltag_ros")) / "cfg" / "tags_41h12.yaml"
    with open(config_path, "r") as file:
        config = yaml.safe_load(file)["apriltag/apriltag"]["ros__parameters"]

    node = Node(
        package="apriltag_ros",
        name="apriltag",
        namespace="apriltag",
        parameters=[config],
        executable="apriltag_node",
    )

    return LaunchDescription([node])