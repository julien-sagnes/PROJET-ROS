import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import launch

def generate_launch_description():

    # Arrêt automatique en cas d'obstacle
    automatic_stop = Node(
        package='projet',
        executable='automatic_stop',
        name='automatic_stop',
        on_exit=launch.actions.Shutdown(),
        output='screen',
        emulate_tty=True,
    )

    # Noeud de contournement d'obstacles
    goal_node = Node(
        package='projet',
        executable='goal_ball',
        name='goal_node',
        output='screen',
        emulate_tty=True,
        on_exit=launch.actions.Shutdown(),
        parameters=[
            {'interface':'/camera/image_raw/compressed'}
        ]
    )


    ld = LaunchDescription()

    # Ajout des actions au lancement
 
    ld.add_action(automatic_stop)
    ld.add_action(goal_node)
    
    return ld


   