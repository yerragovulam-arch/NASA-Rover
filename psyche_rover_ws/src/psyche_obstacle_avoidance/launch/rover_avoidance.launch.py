from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('psyche_obstacle_avoidance')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    obstacle_detector = Node(
        package='psyche_obstacle_avoidance',
        executable='obstacle_detector_node',
        name='obstacle_detector',
        output='screen',
        parameters=[params_file],
    )

    reactive_avoidance = Node(
        package='psyche_obstacle_avoidance',
        executable='reactive_avoidance_node',
        name='reactive_avoidance',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([obstacle_detector, reactive_avoidance])
