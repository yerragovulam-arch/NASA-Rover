from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    obstacle_pkg_share = get_package_share_directory('psyche_obstacle_avoidance')
    obstacle_params_file = os.path.join(obstacle_pkg_share, 'config', 'params.yaml')

    terrain_pkg_share = get_package_share_directory('psyche_terrain_mapping')
    terrain_params_file = os.path.join(terrain_pkg_share, 'config', 'params.yaml')

    obstacle_detector = Node(
        package='psyche_obstacle_avoidance',
        executable='obstacle_detector_node',
        name='obstacle_detector',
        output='screen',
        parameters=[obstacle_params_file],
    )

    reactive_avoidance = Node(
        package='psyche_obstacle_avoidance',
        executable='reactive_avoidance_node',
        name='reactive_avoidance',
        output='screen',
        parameters=[obstacle_params_file],
    )

    terrain_mapper = Node(
        package='psyche_terrain_mapping',
        executable='terrain_mapper_node',
        name='terrain_mapper',
        output='screen',
        parameters=[terrain_params_file],
    )

    path_picker = Node(
        package='psyche_terrain_mapping',
        executable='path_picker_node',
        name='path_picker',
        output='screen',
        parameters=[terrain_params_file],
    )

    state_machine = Node(
        package='psyche_state_machine',
        executable='rover_state_machine_node',
        name='rover_state_machine',
        output='screen',
    )

    return LaunchDescription([
        obstacle_detector,
        reactive_avoidance,
        terrain_mapper,
        path_picker,
        state_machine,
    ])
