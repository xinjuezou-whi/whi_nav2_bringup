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
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    # Input parameters declaration
    # evaluate substitutions at runtime
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_stamped_vel = LaunchConfiguration('use_stamped_vel')
    vehicle = LaunchConfiguration("vehicle").perform(context)
    vehicle_model = LaunchConfiguration("vehicle_model").perform(context)
    use_ekf = LaunchConfiguration("use_ekf").perform(context)
    local_planner = LaunchConfiguration("local_planner").perform(context)
    map = LaunchConfiguration("map")

    if local_planner.lower() in ("dwb", "1"): # in case it is a string
        nav2_params_file_name = f"nav2_params_dwb"
    else:
        nav2_params_file_name = f"nav2_params"
    if use_ekf.lower() in ("true", "1"): # in case it is a string
        nav2_params_file_name = nav2_params_file_name + f"_ekf_{vehicle}.yaml"
    else:
        nav2_params_file_name = nav2_params_file_name + f"_{vehicle}.yaml"
    nav2_params_file = PathJoinSubstitution(
        [FindPackageShare("whi_nav2_bringup"), "config", nav2_params_file_name]
    )
    
    default_bt_xml_file = PathJoinSubstitution(
        [FindPackageShare("whi_nav2_bringup"), "behavior_trees", "navigate_w_replanning_and_recovery_registration.xml"]
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
    
    # rviz visualization
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    launch_nodes = [
        start_whi_motion_hw_if_cmd,
        start_lakibeam1_cmd,
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
            'local_planner', default_value='rpp',
            description='The local planner to use'),
        DeclareLaunchArgument(
            'map', default_value='/home/nvidia/ros2_ws/field_test.yaml',
            description='Full path to map file to load'),
        OpaqueFunction(function=launch_setup)
    ])
