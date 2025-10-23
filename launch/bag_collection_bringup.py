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
    use_stamped_vel = LaunchConfiguration('use_stamped_vel')
    vehicle = LaunchConfiguration("vehicle").perform(context)
    vehicle_model = LaunchConfiguration("vehicle_model").perform(context)
    use_ekf = LaunchConfiguration("use_ekf").perform(context)
    bag_file = LaunchConfiguration("bag_file").perform(context)
    landmark = LaunchConfiguration("landmark").perform(context)

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

    whi_qrcode_pose_launch_file = PathJoinSubstitution([
        FindPackageShare('whi_qrcode_pose'),
        'launch',
        'launch.py'
    ])

    whi_landmark_launch_file = PathJoinSubstitution([
        FindPackageShare('whi_land_marker'),
        'launch',
        'launch.py'
    ])

    # Nodes launching commands
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

    # landmark utility
    start_whi_qrcode_pose_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(whi_qrcode_pose_launch_file),
        condition=IfCondition(LaunchConfiguration("landmark")),
    )

    start_whi_landmark_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(whi_landmark_launch_file),
        condition=IfCondition(LaunchConfiguration("landmark")),
    )

    # bag record
    start_rosbag_record_cmd = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a', '-o', LaunchConfiguration('bag_file')],
        output='screen'
    )

    launch_nodes = [
        start_whi_motion_hw_if_cmd,
        start_lakibeam1_cmd,
        start_rslidar_cmd,
        start_whi_qrcode_pose_cmd,
        start_whi_landmark_cmd,
        # start_rosbag_record_cmd,
    ]

    return launch_nodes

def generate_launch_description():
    return LaunchDescription([
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
            'bag_file', default_value='/home/nvidia/maps/offline',
            description='Output bag file name'),
        DeclareLaunchArgument('landmark', default_value='false',
            description='wether to use landmark'),
        OpaqueFunction(function=launch_setup)
    ])
