import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_share_path
from wcr_launcher.yaml_loader import LauncherConfigurator # type: ignore


def generate_launch_description():
    
    # Paths
    pkg_share = get_package_share_path('wcr_description')
    launcher_share = get_package_share_path('wcr_launcher')
    
    # Extract launch parameters with defaults
    config = LauncherConfigurator()
    namespace = config.namespace
    world_file = config.world_file
    spawn_z_height = config.spawn_z_height
    gz_args = config.gz_args
    
    # Build paths
    world_path = os.path.join(pkg_share, 'worlds', world_file)

    # Include Gazebo (ros_gz_sim)
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': [world_path, TextSubstitution(text=f' {gz_args}')],
            'on_exit_shutdown': 'true',
            'use_sim_time': str(config.use_sim_time).lower(),
        }.items()
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        namespace=namespace,
        arguments=[
            '-topic', f'/{namespace}/robot_description',
            '-name', namespace,
            '-z', str(spawn_z_height)
        ],
    )

    # Bridge (TODO: ako bude potrebe)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{'config_file': os.path.join(launcher_share, 'config', 'bridge.yaml')}],
        namespace=namespace
    )

    return LaunchDescription([
        gazebo,
        spawn, 
        bridge,
    ])
