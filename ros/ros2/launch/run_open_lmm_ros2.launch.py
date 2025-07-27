import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

# Arguments
launch_args = [
    DeclareLaunchArgument(
        name="rviz_enable",
        default_value="false",
        description="visualization"
    ),
    DeclareLaunchArgument(
        name="rviz_config",
        default_value=os.path.join(os.path.dirname(__file__),'../rviz/config/open_lmm.rviz'),
        description="rviz config file path",
    )
]

# Node
def launch_setup(context):
    # print(os.path.dirname(__file__),  # Get the directory of the current file
    #         '../rviz/uni_mapper.rviz'   # Relative path to the rviz config
    #     )
    glim_node = Node(
        package='open_lmm_ros',
        executable='open_lmm_rosnode',
        name='open_lmm_rosnode',
        output="screen",
        emulate_tty=True,
        parameters=[{'use_ansi_code': True}]
    )
    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration("rviz_enable").perform(context)),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments = ['-d', LaunchConfiguration("rviz_config").perform(context)],
    )
    return [glim_node, rviz_node]

# Bag
def bag_setup(context):
    bag_player = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', '/media/gil/DATA/test'],
        output = 'screen'
    )
    bag_delay_player = TimerAction(
        period=3.0,
        actions = [bag_player]
    )
    return [bag_delay_player]

# Main
def generate_launch_description():
    ld = launch.LaunchDescription(launch_args)

    node_setup = OpaqueFunction(function=launch_setup)
    ld.add_action(node_setup)

    # bag = OpaqueFunction(function=bag_setup)
    # ld.add_action(bag)

    return ld
