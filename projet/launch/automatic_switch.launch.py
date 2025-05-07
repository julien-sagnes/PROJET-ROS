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

    # Lancement du noeud line_following
    line_following = Node(
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

    # Noeuds de contournement d'obstacles
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
    
    contourne_obstacles_node = Node(
        package='projet',
        executable='contourne_obstacles',
        name='contourne_obstacles_node',
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
    ld.add_action(line_following)
    ld.add_action(route_obstacle_node)
    ld.add_action(contourne_obstacles_node)
    ld.add_action(cmd_vel_arbiter_node)

    return ld