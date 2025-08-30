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

# Example:
#   $ ros2 launch rslidar_sdk start.py
#
#   SLAM:
#   $ ros2 launch whi_nav2_bringup example_rslidar.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_rtab_rviz = LaunchConfiguration('use_rtab_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    deskewing = LaunchConfiguration('deskewing')
    
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("whi_nav2_bringup"), "launch", "config_mapping_3d.rviz"]
    )

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_rtab_rviz', default_value='true',
            description='Use RTAB map rviz'),

        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        DeclareLaunchArgument(
            'deskewing', default_value='false',
            description='Enable lidar deskewing'),
          
        # Nodes to launch
        Node(
            package='rtabmap_odom', executable='icp_odometry', output='screen',
            parameters=[{
              'frame_id':'laser_multi', # align with rslidar's config
              'odom_frame_id':'odom',
              'wait_for_transform':0.2,
              'expected_update_rate':15.0,
              'deskewing':deskewing,
              'use_sim_time':use_sim_time,
            }],
            remappings=[
              ('scan_cloud', '/rslidar_points')
            ],
            arguments=[
              'Icp/PointToPlane', 'true',
              'Icp/Iterations', '10',
              'Icp/VoxelSize', '0.1',
              'Icp/Epsilon', '0.001',
              'Icp/PointToPlaneK', '20',
              'Icp/PointToPlaneRadius', '0',
              'Icp/MaxTranslation', '2',
              'Icp/MaxCorrespondenceDistance', '1',
              'Icp/Strategy', '1',
              'Icp/OutlierRatio', '0.7',
              'Icp/CorrespondenceRatio', '0.01',
              'Odom/ScanKeyFrameThr', '0.4',
              'OdomF2M/ScanSubtractRadius', '0.1',
              'OdomF2M/ScanMaxSize', '15000',
              'OdomF2M/BundleAdjustment', 'false',
            ]
        ),
            
        Node(
            package='rtabmap_util', executable='point_cloud_assembler', output='screen',
            parameters=[{
              'max_clouds':10,
              'fixed_frame_id':'',
              'use_sim_time':use_sim_time,
            }],
            remappings=[
              ('cloud', 'odom_filtered_input_scan')
            ]
        ),
            
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[{
              'frame_id':'laser_multi', # align with rslidar's config
              'subscribe_depth':False,
              'subscribe_rgb':False,
              'subscribe_scan_cloud':True,
              'approx_sync':False,
              'wait_for_transform':0.2,
              'use_sim_time':use_sim_time,
            }],
            remappings=[
              ('scan_cloud', 'assembled_cloud')
            ],
            arguments=[
              '-d', # This will delete the previous database (~/.ros/rtabmap.db)
              'RGBD/ProximityMaxGraphDepth', '0',
              'RGBD/ProximityPathMaxNeighbors', '1',
              'RGBD/AngularUpdate', '0.05',
              'RGBD/LinearUpdate', '0.05',
              'RGBD/CreateOccupancyGrid', 'false',
              'Mem/NotLinkedNodesKept', 'false',
              'Mem/STMSize', '30',
              'Mem/LaserScanNormalK', '20',
              'Reg/Strategy', '1',
              'Icp/VoxelSize', '0.1',
              'Icp/PointToPlaneK', '20',
              'Icp/PointToPlaneRadius', '0',
              'Icp/PointToPlane', 'true',
              'Icp/Iterations', '10',
              'Icp/Epsilon', '0.001',
              'Icp/MaxTranslation', '3',
              'Icp/MaxCorrespondenceDistance', '1',
              'Icp/Strategy', '1',
              'Icp/OutlierRatio', '0.7',
              'Icp/CorrespondenceRatio', '0.2',
            ]
        ), 
     
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=[{
              'frame_id':'laser_multi', # align with rslidar's config
              'odom_frame_id':'odom',
              'subscribe_odom_info':True,
              'subscribe_scan_cloud':True,
              'approx_sync':False,
              'use_sim_time':use_sim_time,
            }],
            remappings=[
               ('scan_cloud', 'odom_filtered_input_scan')
            ],
            condition=IfCondition(use_rtab_rviz)
        ),
        
		Node(
			package="rviz2",
			executable="rviz2",
			name="rviz2",
			arguments=["-d", rviz_config_file],
            condition=UnlessCondition(use_rtab_rviz),
		),
    ])
