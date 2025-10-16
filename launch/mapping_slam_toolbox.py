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
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
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
    # Input parameters declaration
    # evaluate substitutions at runtime
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    vehicle = LaunchConfiguration("vehicle").perform(context)
    vehicle_model = LaunchConfiguration("vehicle_model").perform(context)
    use_ekf = LaunchConfiguration("use_ekf").perform(context)
    bag_file = LaunchConfiguration("bag_file").perform(context)

    # check running
    if LaunchConfiguration('use_sim_time').perform(context).lower() in ("false", "1"): # in case it is a string
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
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("whi_nav2_bringup"), "launch", "config_mapping.rviz"]
    )
    slam_toolbox_config_file = PathJoinSubstitution(
        [FindPackageShare("whi_nav2_bringup"), "config", "mapper_params_online_async.yaml"]
    )

    # Nodes launching commands
    start_whi_motion_hw_if_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(whi_motion_hw_if_launch_file),
        launch_arguments={
            'vehicle': vehicle,
            'vehicle_model': vehicle_model,
            'use_ekf': use_ekf
        }.items(),
        condition=UnlessCondition(LaunchConfiguration("use_sim_time")),
    )

    # LiDAR
    start_lakibeam1_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_lakibeam1_launch_file),
        launch_arguments={
            'frame_id': 'laser',
            'output_topic0': 'scan',
        }.items(),
        condition=UnlessCondition(LaunchConfiguration("use_sim_time")),
    )

    # slam toolbox
    slam_toolbox_params = RewrittenYaml(
        source_file=slam_toolbox_config_file,
        root_key=namespace,
        param_rewrites={
            'use_sim_time': use_sim_time,
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'scan_topic': 'scan',
            'resolution': '0.05',
        },
        convert_types=True
    )
    start_slam_toolbox_cmd = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_toolbox_params]
    )

    # map server
    start_map_saver_server_cmd = Node(
        package='nav2_map_server',
        executable='map_saver_server',
        output='screen',
        parameters=[slam_toolbox_params]
    )

    # life cycle
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

    # bag play
    start_play_rosbag_cmd = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag_file'), '--clock'],
        condition=IfCondition(LaunchConfiguration("use_sim_time")),
        output='screen',
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
        start_slam_toolbox_cmd,
        start_map_saver_server_cmd,
        start_lifecycle_manager_cmd,
        start_play_rosbag_cmd,
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
        DeclareLaunchArgument('bag_file', default_value='/home/nvidia/maps/offline',
            description='Input bag file name'),
        OpaqueFunction(function=launch_setup)
    ])
