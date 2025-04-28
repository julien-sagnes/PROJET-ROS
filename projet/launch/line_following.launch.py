import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import launch

def generate_launch_description():

    # Lancement de ton noeud principal line_following
    line_following_node = Node(
        package='projet',
        executable='line_following',
        name='line_following_node',
        output='screen',
        emulate_tty=True,
        on_exit=launch.actions.Shutdown(),
        parameters=[
            {'interface':'/camera/image_raw/compressed'}
        ]
    )

   

    ld = LaunchDescription()

    # Ajout des actions au lancement
    ld.add_action(line_following_node)

    return ld


   