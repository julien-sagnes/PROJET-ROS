import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch  # Pour Shutdown()

def generate_launch_description():

    # Active le lidar
    lds_distance = Node(
        package='projet',
        executable='lds_distance',
        name='lds_distance_node',
        on_exit=launch.actions.Shutdown(),
        output='screen',
        emulate_tty=True,
        parameters=[
            {'angle_width_deg':3.0}
        ]
    )

    # Active le suivi de mur
    wall_follower = Node(
        package='projet',
        executable='wall_follower',
        name='wall_follower_node',
        on_exit=launch.actions.Shutdown(),
        output='screen',
        emulate_tty=True,
    )


    ld = LaunchDescription()

    # Ajout des actions au lancement
    ld.add_action(lds_distance)
    ld.add_action(wall_follower)

    return ld