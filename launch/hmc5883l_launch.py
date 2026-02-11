"""Launch file for hmc5883l_compass package."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('hmc5883l_compass')
    default_params = os.path.join(pkg_dir, 'config', 'hmc5883l_params.yaml')

    return LaunchDescription([
        # ── Launch arguments (override on CLI) ────────
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Full path to the parameter YAML file',
        ),

        # ── HMC5883L Compass Node ────────────────────
        Node(
            package='hmc5883l_compass',
            executable='hmc5883l_node.py',
            name='hmc5883l_compass_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
            remappings=[
                ('mag/data', 'mag/data'),
            ],
        ),
    ])
