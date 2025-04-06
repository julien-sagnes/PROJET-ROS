import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch  # Importation nécessaire pour utiliser launch.actions.Shutdown()

def generate_launch_description():
    # Chemin vers le fichier de lancement du monde
    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('projet2025'),
            'launch',
            'projet.launch.py'
        ))
    )

    ld = LaunchDescription()

    # Ajout des actions au lancement
    ld.add_action(world)

    return ld