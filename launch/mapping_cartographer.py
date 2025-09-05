# Copyright 2025 WheelHub Intelligent
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
import os
from ament_index_python.packages import get_package_share_directory
import subprocess

# check if a process name appears in `ps aux`
def is_process_running(name: str) -> bool:
    try:
        # get full command lines
        output = subprocess.check_output(["ps", "-eo", "args"], text=True)
        for line in output.splitlines():
            if name in line:  # substring check is enough for your case
                return True
        return False
    except Exception as e:
        print(f"process check failed: {e}")
        return False

def launch_setup(context, *args, **kwargs):
    unique_processes = [
        "whi_rc_bridge_node",
        "whi_indicators_node",
        "whi_modbus_io_node",
        "whi_imu_node",
        "whi_battery_monitor_node",
    ]
    for exe in unique_processes:
        if is_process_running(exe):
            print(f"running process detected: {exe} is already running. Aborting launch.")
            return []

    # Input parameters declaration
    # evaluate substitutions at runtime
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    vehicle = LaunchConfiguration("vehicle").perform(context)
    vehicle_model = LaunchConfiguration("vehicle_model").perform(context)
    use_ekf = LaunchConfiguration("use_ekf").perform(context)
    use_3d = LaunchConfiguration("use_3d").perform(context)

    # Getting directories and launch-files
    whi_motion_hw_if_launch_file = PathJoinSubstitution([
        FindPackageShare('whi_motion_hw_interface'),
        'launch',
        'bringup.py'
    ])
    lidar_lakibeam1_launch_file = PathJoinSubstitution([
        FindPackageShare('lakibeam1'),
        'launch',
        'lakibeam1_scan.launch.py'
    ])
    lidar_rslidar_launch_file = PathJoinSubstitution([
        FindPackageShare('rslidar_sdk'),
        'launch',
        'start.py'
    ])

    if use_ekf.lower() in ("true", "1"): # in case it is a string
        odom_remap = ('/odom', '/odometry/filtered')
    else:
        odom_remap = ('/odom', '/odom')

    cartographer_config_dir = os.path.join(get_package_share_directory('whi_nav2_bringup'), 'config')
    if use_3d.lower() in ("true", "1"): # in case it is a string
        cartographer_config_file = "backpack_3d.lua"
        rviz_config_file = PathJoinSubstitution(
            [FindPackageShare("whi_nav2_bringup"), "launch", "config_mapping_3d.rviz"])
        remaps=[('/points2', '/rslidar_points'),
                ('/imu', '/imu_data'),
                odom_remap]
    else:
        cartographer_config_file = "backpack_2d.lua"
        rviz_config_file = PathJoinSubstitution(
            [FindPackageShare("whi_nav2_bringup"), "launch", "config_mapping.rviz"])
        remaps=[odom_remap]

    # Nodes launching commands
    start_whi_motion_hw_if_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(whi_motion_hw_if_launch_file),
        launch_arguments={
            'vehicle': vehicle,
            'vehicle_model': vehicle_model,
            'use_ekf': use_ekf
        }.items()
    )

    # LiDAR
    start_lakibeam1_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_lakibeam1_launch_file),
        launch_arguments={
            'frame_id': 'laser',
            'output_topic0': 'scan',
        }.items(),
        condition=UnlessCondition(LaunchConfiguration("use_3d"))
    )

    start_rslidar_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_rslidar_launch_file),
        launch_arguments={
            'start_rviz': 'false',
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_3d"))
    )

    # cartographer
    start_cartographer_cmd = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', cartographer_config_dir,
                   '-configuration_basename', cartographer_config_file],
        remappings=remaps,
    )
    
    start_occupancy_grid_cmd = Node(
        package='cartographer_ros',
        executable='occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', str(0.05), '-publish_period_sec', str(1.0)],
    )

    # map server
    start_map_saver_server_cmd = Node(
        package='nav2_map_server',
        executable='map_saver_server',
        output='screen',
        parameters=[
            {'save_map_timeout': 10},
            {'free_thresh_default': 0.25},
            {'occupied_thresh_default': 0.65}]
    )

    lifecycle_nodes = ['map_saver']
    start_lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes}]
    )

    # rviz visualization
    start_rviz_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
    )

    launch_nodes = [
        start_whi_motion_hw_if_cmd,
        start_lakibeam1_cmd,
        start_rslidar_cmd,
        start_cartographer_cmd,
        start_occupancy_grid_cmd,
        start_map_saver_server_cmd,
        start_lifecycle_manager_cmd,
        start_rviz_cmd
    ]

    return launch_nodes

def generate_launch_description():
    return LaunchDescription([
        # Declare the launch arguments
        DeclareLaunchArgument('namespace', default_value='',
            description='Top-level namespace'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('autostart', default_value='true',
            description='Automatically startup the nav2 stack'),
        DeclareLaunchArgument("vehicle", default_value="L1",
            description="the mobile robot series"),
        DeclareLaunchArgument("vehicle_model", default_value="diff",
            description="the mobile robot's dynamic model"),
        DeclareLaunchArgument('use_ekf', default_value='true',
            description='Use ekf to fuse localization'),
        DeclareLaunchArgument('use_3d', default_value='false',
            description='Use multi lidar for 3D mapping'),
        OpaqueFunction(function=launch_setup)
    ])
