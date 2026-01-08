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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
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
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_stamped_vel = LaunchConfiguration('use_stamped_vel')
    vehicle = LaunchConfiguration("vehicle").perform(context)
    vehicle_model = LaunchConfiguration("vehicle_model").perform(context)
    use_ekf = LaunchConfiguration("use_ekf").perform(context)
    local_planner = LaunchConfiguration("local_planner").perform(context)
    map = LaunchConfiguration("map")
    load_state_file = LaunchConfiguration("load_state_file")
    use_rtabmap = LaunchConfiguration("use_rtabmap")
    db_file = LaunchConfiguration("db_file")
    keepout_mask_file = LaunchConfiguration("keepout_mask_file")

    if local_planner.lower() in ("dwb", "1"): # in case it is a string
        nav2_params_file_name = f"nav2_params_dwb_{vehicle}.yaml"
    elif local_planner.lower() in ("mppi", "1"): # in case it is a string
        nav2_params_file_name = f"nav2_params_mppi_{vehicle}.yaml"
    elif local_planner.lower() in ("rpp", "1"): # in case it is a string
        nav2_params_file_name = f"nav2_params_rpp_{vehicle}.yaml"
    else:
        nav2_params_file_name = f"nav2_params_dwb"

    nav2_params_file=os.path.join(get_package_share_directory('whi_nav2_bringup'),
        'config', nav2_params_file_name)

    whi_nav2_bringup_launch_file = PathJoinSubstitution([
        FindPackageShare('whi_nav2_bringup'),
        'launch',
        'nav2_bringup.py'
    ])

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

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("whi_nav2_bringup"), "launch", "config_nav2.rviz"]
    )

    # Nodes launching commands
    start_nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(whi_nav2_bringup_launch_file),
        launch_arguments={
            'map': map,
            'load_state_file': load_state_file,
            'use_rtabmap': use_rtabmap,
            'db_file': db_file,
            'use_ekf': use_ekf,
            'vehicle': vehicle,
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'use_composition': 'False',
            'use_respawn': 'False',
            'keepout_mask_file': keepout_mask_file,
        }.items(),
    )

    start_whi_motion_hw_if_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(whi_motion_hw_if_launch_file),
        launch_arguments={
            'use_stamped_vel': use_stamped_vel,
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
        }.items()
    )

    start_rslidar_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_rslidar_launch_file),
        launch_arguments={
            'start_rviz': 'false',
        }.items()
    )

    # pose registration
    start_pose_registration_cmd = Node(
        package='whi_pose_registration_server',
        executable='whi_pose_registration_server',
        name='whi_pose_registration_server',
        parameters=[nav2_params_file],
        output='screen'
    )

    # bt actions server
    start_bt_actions_server_cmd = Node(
        package='whi_nav2_bt_actions_server',
        executable='whi_nav2_bt_actions_server',
        name='whi_nav2_bt_actions_server',
        parameters=[nav2_params_file],
        output='screen'
    )
    
    # life cycle nodes
    lifecycle_nodes = [
        'whi_pose_registration_server',
        'whi_nav2_bt_actions_server',
    ]
    start_life_cycle_nodes_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_whi',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': lifecycle_nodes}
        ]
    )
    
    # rviz visualization
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    launch_nodes = [
        start_pose_registration_cmd,
        start_bt_actions_server_cmd,
        start_life_cycle_nodes_cmd,
        start_whi_motion_hw_if_cmd,
        start_lakibeam1_cmd,
        start_rslidar_cmd,
        start_nav2_bringup_cmd,
        start_rviz_cmd
    ]

    return launch_nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'use_stamped_vel', default_value='false', # false for foxy
            description='Use stamped twist'),
        DeclareLaunchArgument(
            "vehicle", default_value="L1",
            description="the mobile robot series"),
        DeclareLaunchArgument(
            "vehicle_model", default_value="diff",
            description="the mobile robot's dynamic model"),
        DeclareLaunchArgument(
            'use_ekf', default_value='true',
            description='Use ekf to fuse localization'),
        DeclareLaunchArgument(
            'local_planner', default_value='dwb',
            description='The local planner to use'),
        DeclareLaunchArgument(
            'map', default_value='/home/nvidia/ros2_ws/field_test.yaml',
            description='Full path to map file to load'),
        DeclareLaunchArgument(
            'load_state_file', default_value='',
            description='Full path to pbstream file to load will trigger cartographer localization'),
        DeclareLaunchArgument(
            'use_rtabmap', default_value='false',
            description='Use rtabmap to localizing'),
        DeclareLaunchArgument(
            'db_file', default_value='/home/nvidia/.ros/rtabmap.db',
            description='Full path to database file to load will trigger rtabmap localization'),
        DeclareLaunchArgument(
            'keepout_mask_file', default_value='',
            description='Full path to keepout zone file to load'),
        OpaqueFunction(function=launch_setup)
    ])
