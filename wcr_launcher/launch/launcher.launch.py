import os
from ament_index_python.packages import get_package_share_path, get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from wcr_launcher.yaml_loader import LauncherConfigurator

def generate_launch_description():

    # Paths and configuration
    pkg_share = get_package_share_path('wcr_description')
    controller_share = get_package_share_path('wcr_control')
    config = LauncherConfigurator()

    # Robot State Publisher (CENTRAL)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=config.namespace,
        output="both",
        parameters=[{
            'robot_description': config.build_robot_description(),
            'use_sim_time': config.use_sim_time,
            "namespace" : config.namespace,
        }]
    )

    # Include Gazebo launch (conditional)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'gazebo.launch.py')
        ),
        condition=IfCondition(str(config.sim).lower())
    )

    # Include controllers launch (conditional)
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_share, 'launch', 'controller.launch.py')
        ),
        condition=IfCondition(str(config.use_controllers).lower())
    )

    controller_launch = Node(
        package="wcr_controllers",
        executable="inv_kin_controller",
        name="inv_kin_controller",
        namespace=config.namespace,
    )

    # ROSBridge server (for Visualization)
    rosbridge_server = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rosbridge_server'), 'launch', 'rosbridge_websocket_launch.xml')
        )
    )

    # Teleop cmd vel to controller node
    keyboard_node = Node(
        package="wcr_keyboard",
        executable="twist_to_controller",
        name="twist_to_controller",
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo_launch,
        control_launch,
        controller_launch,
        rosbridge_server,
        #keyboard_node
    ])