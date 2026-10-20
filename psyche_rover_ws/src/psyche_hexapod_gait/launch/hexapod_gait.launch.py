from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('psyche_hexapod_gait')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    gait_controller = Node(
        package='psyche_hexapod_gait',
        executable='gait_controller_node',
        name='gait_controller',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([gait_controller])
