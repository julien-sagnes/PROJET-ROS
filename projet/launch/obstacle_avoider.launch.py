import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import launch

def generate_launch_description():

    # Lancement du noeud line_following
    contourne_obstacles_node = Node(
        package='projet',
        executable='contourne_obstacles',
        name='contourne_obstacles_node',
        on_exit=launch.actions.Shutdown(),
        output='screen',
        emulate_tty=True,
    )

    # Arrêt automatique en cas d'obstacle
    automatic_stop = Node(
        package='projet',
        executable='automatic_stop',
        name='automatic_stop',
        on_exit=launch.actions.Shutdown(),
        output='screen',
        emulate_tty=True,
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

    line_obstacles_node = Node(
        package='projet',
        executable='line_obstacle',
        name='line_obstacles_node',
        output='screen',
        emulate_tty=True,
        on_exit=launch.actions.Shutdown(),
        parameters=[
            {'interface':'/camera/image_raw/compressed'}
        ]
    )


    ld = LaunchDescription()

    # Ajout des actions au lancement
    ld.add_action(lds_distance) 
    ld.add_action(automatic_stop)
    ld.add_action(contourne_obstacles_node)
    ld.add_action(line_obstacles_node)
    

    return ld


   