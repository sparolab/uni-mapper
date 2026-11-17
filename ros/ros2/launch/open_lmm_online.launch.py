"""Online OpenLMM (phase 1): stream FAST-LIO keyframes, re-merge periodically.

Each agent needs `<topic_prefix>/<agent>/<cloud_topic>` (body-frame cloud) and
`<topic_prefix>/<agent>/<odom_topic>` with matching stamps, e.g. FAST-LIO's
`/cloud_registered_body` and `/Odometry` remapped per robot (see
`fast_lio_multi.launch.py`).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def _nodes(context):
    get = lambda name: LaunchConfiguration(name).perform(context)
    agents = [a for a in get("agents").replace(",", " ").split() if a]
    frame_id = get("rviz_frame_id")
    rviz_config = os.path.join(
        get_package_share_directory("open_lmm_ros"), "rviz", "open_lmm.rviz"
    )
    online = Node(
        package="open_lmm_ros",
        executable="open_lmm_online_node",
        name="open_lmm_ros",  # matches the topics in rviz/open_lmm.rviz
        output="screen",
        parameters=[
            {
                "config_path": get("config_path"),
                "work_dir": get("work_dir"),
                "agents": agents,
                "topic_prefix": get("topic_prefix"),
                "cloud_topic": get("cloud_topic"),
                "odom_topic": get("odom_topic"),
                "merge_period_s": float(get("merge_period_s")),
                "min_keyframes_per_agent": int(get("min_keyframes_per_agent")),
                "min_agents": int(get("min_agents")),
                "keyframe_translation_m": float(get("keyframe_translation_m")),
                "keyframe_rotation_deg": float(get("keyframe_rotation_deg")),
                "scans_per_keyframe": int(get("scans_per_keyframe")),
                "keyframe_voxel_m": float(get("keyframe_voxel_m")),
                "rviz_visualization_enabled": True,
                "rviz_frame_id": frame_id,
                "rviz_preview_voxel_size_m": float(get("rviz_preview_voxel_size_m")),
            }
        ],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="open_lmm_rviz",
        output="screen",
        condition=IfCondition(get("use_rviz")),
        arguments=["-d", rviz_config, "-f", frame_id],
    )
    return [online, rviz]


def generate_launch_description():
    defaults = {
        "config_path": "config",
        "work_dir": "/tmp/open_lmm_online",
        "agents": "agent1 agent2",
        "topic_prefix": "",
        "cloud_topic": "cloud_registered_body",
        "odom_topic": "Odometry",
        "merge_period_s": "60.0",
        "min_keyframes_per_agent": "20",
        "min_agents": "2",
        "keyframe_translation_m": "1.0",
        "keyframe_rotation_deg": "10.0",
        "scans_per_keyframe": "5",
        "keyframe_voxel_m": "0.1",
        "rviz_frame_id": "map",
        "rviz_preview_voxel_size_m": "0.4",
        "use_rviz": "true",
    }
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
        + [OpaqueFunction(function=_nodes)]
    )
