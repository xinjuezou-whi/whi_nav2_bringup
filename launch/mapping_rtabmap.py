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
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
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
    enable_odom = LaunchConfiguration("enable_odom").perform(context)

    # Getting directories and launch-files
    whi_motion_hw_if_launch_file = PathJoinSubstitution([
        FindPackageShare('whi_motion_hw_interface'),
        'launch',
        'bringup.py'
    ])
    lidar_rslidar_launch_file = PathJoinSubstitution([
        FindPackageShare('rslidar_sdk'),
        'launch',
        'start.py'
    ])
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("whi_nav2_bringup"), "launch", "config_mapping_3d.rviz"]
    )

    # Nodes launching commands
    start_whi_motion_hw_if_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(whi_motion_hw_if_launch_file),
        launch_arguments={
            'vehicle': vehicle,
            'vehicle_model': vehicle_model,
            'use_ekf': use_ekf,
            'enable_odom': enable_odom
        }.items()
    )

    # LiDAR
    start_rslidar_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_rslidar_launch_file),
        launch_arguments={
            'start_rviz': 'false',
        }.items()
    )

    # rtab map
    start_rtabmap_odom_cmd = Node(
        package='rtabmap_odom', executable='icp_odometry', output='screen',
        parameters=[{
            'frame_id': 'laser_multi', # align with rslidar's config
            'odom_frame_id': 'odom',
            'wait_for_transform': 0.2,
            'expected_update_rate': 15.0,
            'deskewing': False,
            'use_sim_time': use_sim_time,
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
        ],
        condition=UnlessCondition(LaunchConfiguration("enable_odom")),
    )
            
    start_rtabmap_util_cmd = Node(
        package='rtabmap_util', executable='point_cloud_assembler', output='screen',
        parameters=[{
            'max_clouds': 10,
            'fixed_frame_id': '',
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('cloud', 'odom_filtered_input_scan')
        ],
        condition=UnlessCondition(LaunchConfiguration("enable_odom")),
    )

    start_rtabmap_slam_ipc_cmd = Node(
        package='rtabmap_slam', executable='rtabmap', output='screen',
        parameters=[{
            'frame_id': 'laser_multi', # align with rslidar's config
            'subscribe_depth': False,
            'subscribe_rgb': False,
            'subscribe_scan_cloud': True,
            'approx_sync': False,
            'wait_for_transform': 0.2,
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('scan_cloud', 'assembled_cloud')
        ],
        arguments=[
            '-d', # This will delete the previous database (~/.ros/rtabmap.db)
            'Grid/3D', 'false',
            'Grid/CellSize', '0.1',
            'Grid/RayTracing', 'true',
            'Grid/RayTracingRange', '6.0',
            'Grid/ClusterRadius', '0.1',
            'Grid/MinClusterSize', '20',
            'Grid/ScanVoxelSize', '0.05',
            'Grid/RangeMin', '0.6',
            'Grid/RangeMax', '80.0',
            'Grid/MaxGroundHeight', '0.2',
            'Grid/MinGroundHeight', '-0.2',
            'Grid/FlatObstacleDetected', 'true',
            'Reg/Force3DoF', 'true',
            'Optimizer/Iterations', '30',
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
        ],
        condition=UnlessCondition(LaunchConfiguration("enable_odom")),
    )

    start_rtabmap_slam_cmd = Node(
        package='rtabmap_slam', executable='rtabmap', output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'subscribe_depth': False,
            'subscribe_rgb': False,
            'subscribe_scan_cloud': True,
            'approx_sync': True,
            'wait_for_transform': 0.2,
            'queue_size': 30, # increase from default 10
            'use_sim_time': use_sim_time,
            'Grid/3D': 'false',
            'Grid/CellSize': '0.1',
            'Grid/RayTracing': 'true',
            'Grid/RayTracingRange': '6.0',
            'Grid/ClusterRadius': '0.1',
            'Grid/MinClusterSize': '20',
            'Grid/ScanVoxelSize': '0.05',
            'Grid/RangeMin': '0.6',
            'Grid/RangeMax': '80.0',
            'Grid/MaxGroundHeight': '0.2',
            'Grid/MinGroundHeight': '-0.2',
            'Grid/FlatObstacleDetected': 'true',
            'Reg/Force3DoF': 'true',
            'Optimizer/Iterations': '30',
            'Icp/PointToPlane': 'true',
            'Icp/MaxCorrespondenceDistance': '2.0',
            'Icp/Iterations': '20',
        }],
        remappings=[
            ('scan_cloud', '/rslidar_points')
        ],
        condition=IfCondition(LaunchConfiguration("enable_odom")),
    )

    # map server
    start_map_saver_server_cmd = Node(
        package='nav2_map_server',
        executable='map_saver_server',
        output='screen',
        parameters=[
            {'save_map_timeout': 20},
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
        start_rslidar_cmd,
        start_rtabmap_odom_cmd,
        start_rtabmap_util_cmd,
        start_rtabmap_slam_ipc_cmd,
        start_rtabmap_slam_cmd,
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
        DeclareLaunchArgument('deskewing', default_value='false',
            description='Enable lidar deskewing'),
        DeclareLaunchArgument('enable_odom', default_value='true',
            description='whether to publish odom'),
        OpaqueFunction(function=launch_setup)
    ])
