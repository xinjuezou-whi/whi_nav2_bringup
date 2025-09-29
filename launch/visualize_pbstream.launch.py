"""
  Copyright 2018 The Cartographer Authors
  Copyright 2022 Wyca Robotics (for the ros2 conversion)

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import Shutdown
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ## ***** Launch arguments *****
    pbstream_filename_arg = DeclareLaunchArgument('pbstream_filename')

    cartographer_config_dir = os.path.join(get_package_share_directory('whi_nav2_bringup'), 'config')

    ## ***** Nodes *****
    rviz_node = Node(
        package = 'rviz2',
        executable = 'rviz2',
        on_exit = Shutdown(),
        arguments = ['-d', FindPackageShare('cartographer_ros').find('cartographer_ros') + '/configuration_files/demo_2d.rviz'],
    )

    cartographer_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_node',
        arguments = [
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', 'visualize_pbstream.lua',
            '-load_state_filename', LaunchConfiguration('pbstream_filename'),
            '-load_frozen_state=false',
            '-start_trajectory_with_default_topics=false'],
        output = 'screen'
        )

    return LaunchDescription([
        # Launch arguments
        pbstream_filename_arg,
        # Nodes
        rviz_node,
        cartographer_node
    ])
