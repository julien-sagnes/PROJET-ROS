import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import launch
from numpy import pi

def generate_launch_description():
    # Déclare les arguments pour x, y et yaw (pour que le robot se positionne au mur)
    x_pose = DeclareLaunchArgument('x_pose', default_value='-0.82', description='x coordinate of robot')
    y_pose = DeclareLaunchArgument('y_pose', default_value='-0.50', description='y coordinate of robot')
    yaw_angle = DeclareLaunchArgument('yaw_angle', default_value=str(3*pi/2), description='yaw angle of robot')

    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('projet2025'),
            'launch',
            'projet.launch.py'
        )),
        launch_arguments={
            'x_pose': LaunchConfiguration('x_pose'),
            'y_pose': LaunchConfiguration('y_pose'),
            'yaw_angle': LaunchConfiguration('yaw_angle')
        }.items()
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

    # Définir d'autres actions comme le lancement de nœuds ou de commandes spécifiques
    ld = LaunchDescription()

    # Ajouter les actions
    ld.add_action(x_pose)
    ld.add_action(y_pose)
    ld.add_action(yaw_angle)
    ld.add_action(world)
    ld.add_action(lds_distance)

    return ld