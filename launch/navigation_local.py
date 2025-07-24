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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    vehicle = LaunchConfiguration("vehicle")
    vehicle_model = LaunchConfiguration("vehicle_model")
    map = LaunchConfiguration("map")
    
    nav2_params_file = PathJoinSubstitution(
        [FindPackageShare("whi_nav2_bringup"), "config", "nav2_params_E1.yaml"]
    )
    
    default_bt_xml_file = PathJoinSubstitution(
        [FindPackageShare("nav2_bt_navigator"), "behavior_trees", "navigate_w_replanning_and_recovery.xml"]
    )

    nav2_bringup_launch_file = PathJoinSubstitution([
        FindPackageShare('nav2_bringup'),
        'launch',
        'bringup_launch.py'
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

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("whi_nav2_bringup"), "launch", "config_nav2.rviz"]
    )

    # Nodes launching commands
    start_nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_bringup_launch_file),
        launch_arguments={
            'map': map,
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'default_bt_xml_filename': default_bt_xml_file}.items(),
    )

    start_whi_motion_hw_if_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(whi_motion_hw_if_launch_file),
        launch_arguments={
            'vehicle': vehicle,
            'vehicle_model': vehicle_model,
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
    
    # rviz visualization
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'map', default_value='/home/nvidia/ros2_ws/field_test.yaml',
            description='Full path to map file to load'),
        DeclareLaunchArgument(
            'nav2_params', default_value=nav2_params_file,
            description='Full path to param file to load'),
        DeclareLaunchArgument(
            "vehicle", default_value="L1",
            description="the mobile robot series"),
        DeclareLaunchArgument(
            "vehicle_model", default_value="diff",
            description="the mobile robot's dynamic model"),
        start_whi_motion_hw_if_cmd,
        start_lakibeam1_cmd,
        start_nav2_bringup_cmd,
        start_rviz_cmd
    ])
