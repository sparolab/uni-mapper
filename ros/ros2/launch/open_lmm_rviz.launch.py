from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    config_path = LaunchConfiguration("config_path")
    frame_id = LaunchConfiguration("rviz_frame_id")
    voxel_size = LaunchConfiguration("rviz_preview_voxel_size_m")
    maximum_points = LaunchConfiguration("rviz_max_point_count")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = os.path.join(
        get_package_share_directory("open_lmm_ros"), "rviz", "open_lmm.rviz"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_path", default_value="config"),
            DeclareLaunchArgument("rviz_frame_id", default_value="map"),
            DeclareLaunchArgument(
                "rviz_preview_voxel_size_m", default_value="0.4"
            ),
            DeclareLaunchArgument("rviz_max_point_count", default_value="2000000"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            Node(
                package="open_lmm_ros",
                executable="open_lmm_rosnode",
                name="open_lmm_ros",
                output="screen",
                parameters=[
                    {
                        "config_path": config_path,
                        "rviz_visualization_enabled": True,
                        "rviz_frame_id": frame_id,
                        "rviz_preview_voxel_size_m": ParameterValue(
                            voxel_size, value_type=float
                        ),
                        "rviz_max_point_count": ParameterValue(
                            maximum_points, value_type=int
                        ),
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="open_lmm_rviz",
                output="screen",
                condition=IfCondition(use_rviz),
                arguments=["-d", rviz_config, "-f", frame_id],
            ),
        ]
    )
