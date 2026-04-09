import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. Start Robot Hardware
    whi_hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('whi_motion_hw_interface'), 'launch', 'bringup.py')
        ]),
        launch_arguments={'vehicle': 'E2', 'vehicle_model': 'swerve', 'start_rviz': 'false'}.items()
    )

    # 2. Start Lakibeam Lidar
    lidar_lakibeam1_launch_file = PathJoinSubstitution([
        FindPackageShare('lakibeam1'),
        'launch',
        'lakibeam1_scan.launch.py'
    ])
    start_lakibeam1_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_lakibeam1_launch_file),
        launch_arguments={
            'frame_id': 'laser',
            'output_topic0': 'scan',
        }.items(),
    )

    lidar_docking_params_file = PathJoinSubstitution([
        get_package_share_directory('whi_nav2_bringup'),
        'config',
        'lidar_docking_params.yaml'
    ])
    docking_node = Node(
        package='whi_nav2_bringup',
        executable='docking_node.py',
        name='lidar_docking',
        parameters=[lidar_docking_params_file],
        output='screen'
    )

    # 3. Rviz2
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("whi_nav2_bringup"), "launch", "config_docking.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
    )

    return LaunchDescription([
        whi_hardware,
        start_lakibeam1_cmd,
        docking_node,
        # rviz_node
    ])