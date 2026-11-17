"""One FAST-LIO (thirdparty/FAST_LIO_ROS2) instance per robot for OpenLMM online.

FAST-LIO publishes absolute topics (/cloud_registered_body, /Odometry, ...),
so every instance is remapped to /<agent>/... . Example:

  ros2 launch open_lmm_ros fast_lio_multi.launch.py \
      agents:="agent1 agent2" \
      lidar_topics:="/robot1/velodyne_points /robot2/velodyne_points" \
      imu_topics:="/robot1/imu/data /robot2/imu/data" \
      fast_lio_config:=velodyne.yaml

Then start open_lmm_online.launch.py with the same `agents`.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def _split(value):
    return [v for v in value.replace(",", " ").split() if v]


def _nodes(context):
    get = lambda name: LaunchConfiguration(name).perform(context)
    agents = _split(get("agents"))
    lidars = _split(get("lidar_topics"))
    imus = _split(get("imu_topics"))
    configs = _split(get("fast_lio_config"))
    if len(lidars) != len(agents) or len(imus) != len(agents):
        raise RuntimeError("agents, lidar_topics and imu_topics need equal length")
    if len(configs) == 1:
        configs = configs * len(agents)
    share = get_package_share_directory("fast_lio")
    nodes = []
    for agent, lidar, imu, config in zip(agents, lidars, imus, configs):
        config_path = config if config.startswith("/") else os.path.join(
            share, "config", config)
        nodes.append(
            Node(
                package="fast_lio",
                executable="fastlio_mapping",
                name=f"fast_lio_{agent}",
                output="screen",
                parameters=[
                    config_path,
                    {
                        "common.lid_topic": lidar,
                        "common.imu_topic": imu,
                        "publish.scan_bodyframe_pub_en": True,
                        "publish.dense_publish_en": True,
                        "pcd_save.pcd_save_en": False,
                    },
                ],
                remappings=[
                    ("/cloud_registered_body", f"/{agent}/cloud_registered_body"),
                    ("/cloud_registered", f"/{agent}/cloud_registered"),
                    ("/cloud_effected", f"/{agent}/cloud_effected"),
                    ("/Laser_map", f"/{agent}/Laser_map"),
                    ("/Odometry", f"/{agent}/Odometry"),
                    ("/path", f"/{agent}/path"),
                    ("/tf", f"/{agent}/tf"),
                    ("/tf_static", f"/{agent}/tf_static"),
                ],
            )
        )
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("agents", default_value="agent1 agent2"),
            DeclareLaunchArgument("lidar_topics"),
            DeclareLaunchArgument("imu_topics"),
            DeclareLaunchArgument("fast_lio_config", default_value="velodyne.yaml"),
            OpaqueFunction(function=_nodes),
        ]
    )
