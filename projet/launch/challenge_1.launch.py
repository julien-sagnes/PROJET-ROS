import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch  # Pour Shutdown()

def generate_launch_description():
    # Chemin vers le fichier de lancement du monde
    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('projet2025'),
            'launch',
            'projet.launch.py'
        ))
    )

    # Active le lidar
    lds_distance = Node(
        package='projet',
        executable='lds_distance',
        name='lds_distance_node',
        on_exit=launch.actions.Shutdown(),
        output='screen',
        emulate_tty=True,
    )

    # Arrêt automatique en cas d'obstacle
    automatic_stop = Node(
        package='projet',
        executable='automatic_stop',
        name='automatic_stop_node',
        on_exit=launch.actions.Shutdown(),
        output='screen',
        emulate_tty=True,
    )

    ld = LaunchDescription()

    # Ajout des actions au lancement
    ld.add_action(world)
    ld.add_action(lds_distance)
    ld.add_action(automatic_stop)

    return ld