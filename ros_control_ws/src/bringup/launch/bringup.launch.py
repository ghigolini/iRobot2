from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import yaml
import os

def generate_launch_description():
    bringup_share = get_package_share_directory('bringup')
    config_path = os.path.join(bringup_share, 'config', 'config.yaml')

    with open(config_path, 'r') as f:
        common_params = yaml.safe_load(f)

    planner = Node(
        package='planner',
        executable='planner',
        name='planner',
        output='screen',
        emulate_tty=True,
        parameters=[common_params],  # pass dict
        arguments=['--ros-args', '--log-level', 'INFO']
    )

    actuator = Node(
        package='actuator',
        executable='actuator',
        name='actuator',
        output='screen',
        emulate_tty=True,
        parameters=[common_params],
        arguments=['--ros-args', '--log-level', 'INFO']
    )

    maze_solver = Node(
        package='maze_solver',
        executable='maze_solver',
        name='maze_solver',
        output='screen',
        emulate_tty=True,
        parameters=[common_params],
        arguments=['--ros-args', '--log-level', 'INFO']
    )

    voice_node = Node(
        package='voice_node',
        executable='voice_node',
        name='voice_node',
        output='screen',
        emulate_tty=True,
        parameters=[common_params],
        arguments=['--ros-args', '--log-level', 'INFO']
    )

    return LaunchDescription([planner, actuator, maze_solver])
