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
    route_obstacle_node = Node(
        package='projet',
        executable='route_obstacle',
        name='route_obstacle_node',
        output='screen',
        emulate_tty=True,
        on_exit=launch.actions.Shutdown(),
        parameters=[
            {'interface':'/camera/image_raw/compressed'}
        ]
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

    wall_derniere_chance = Node(
        package='projet',
        executable='wall_derniere_chance',
        name='wall_derniere_chance_node',
        output='screen',
        emulate_tty=True,
        on_exit=launch.actions.Shutdown(),
    )

    cmd_vel_arbiter_node = Node(
        package='projet',
        executable='cmd_vel_arbiter',
        name='cmd_vel_arbiter_node',
        output='screen',
        emulate_tty=True,
        on_exit=launch.actions.Shutdown(),
    )

    ld = LaunchDescription()

    # Ajout des actions au lancement
    ld.add_action(lds_distance)
    ld.add_action(automatic_stop)
    ld.add_action(wall_derniere_chance)
    ld.add_action(route_obstacle_node)
    ld.add_action(cmd_vel_arbiter_node)
    

    return ld


   