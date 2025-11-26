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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
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
    # Input parameters declaration
    # evaluate substitutions at runtime
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    vehicle = LaunchConfiguration("vehicle").perform(context)
    vehicle_model = LaunchConfiguration("vehicle_model").perform(context)
    use_ekf = LaunchConfiguration("use_ekf").perform(context)
    icp_odom = LaunchConfiguration("icp_odom").perform(context)
    incremental = LaunchConfiguration("incremental").perform(context)
    bag_file = LaunchConfiguration("bag_file").perform(context)
    landmark = LaunchConfiguration("landmark").perform(context)
    rgb = LaunchConfiguration("rgb").perform(context)
    create_dict = LaunchConfiguration("create_dict").perform(context)
    fixed_dict = LaunchConfiguration("fixed_dict").perform(context)

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
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("whi_nav2_bringup"), "launch", "config_mapping_3d.rviz"]
    )

    # Nodes launching commands
    enable_odom = 'false' if icp_odom.lower() == 'true' else 'true'
    start_whi_motion_hw_if_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(whi_motion_hw_if_launch_file),
        launch_arguments={
            'vehicle': vehicle,
            'vehicle_model': vehicle_model,
            'use_ekf': use_ekf,
            'enable_odom': enable_odom
        }.items(),
        condition=UnlessCondition(LaunchConfiguration("use_sim_time")),
    )

    # LiDAR
    start_rslidar_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_rslidar_launch_file),
        launch_arguments={
            'start_rviz': 'false',
        }.items(),
        condition=UnlessCondition(LaunchConfiguration("use_sim_time")),
    )

    # RGB camera
    start_usb_cam_cam =  Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[
            '/home/nvidia/ros2_humble/src/usb_cam/config/params_hik.yaml'
        ],
        condition=IfCondition(LaunchConfiguration("rgb")),
    )

    # landmark utility
    start_whi_qrcode_pose_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(whi_qrcode_pose_launch_file),
        condition=IfCondition(
            PythonExpression([
                '"', LaunchConfiguration('landmark'), '"', ' == "true" and ',
                '"', LaunchConfiguration('use_sim_time'), '"', ' == "false"'
            ])
        )
    )

    start_whi_landmark_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(whi_landmark_launch_file),
        condition=IfCondition(
            PythonExpression([
                '"', LaunchConfiguration('landmark'), '"', ' == "true" and ',
                '"', LaunchConfiguration('use_sim_time'), '"', ' == "false"'
            ])
        )
    )

    # rtab map
    parameters_odom={
        'Odom/Strategy': '0',                     # 0=Frame-to-Map (F2M) 1=Frame-to-Frame (F2F) 2=Fovis 3=viso2 4=DVO-SLAM 5=ORB_SLAM2 6=OKVIS 7=LOAM 8=MSCKF_VIO 9=VINS-Fusion 10=OpenVINS 11=FLOAM 12=Open3D
        'Odom/GuessMotion': 'true',
        'Odom/ScanKeyFrameThr': '0.4',
        'Odom/Force3DoF': 'true',
        'OdomF2M/ScanSubtractRadius': '0.1',      # Radius used to filter points of a new added scan to local map. This could match the voxel size of the scans
        'OdomF2M/ScanMaxSize': '15000',
        'OdomF2M/BundleAdjustment': '3',          # Local bundle adjustment: 0=disabled, 1=g2o, 2=cvsba, 3=Ceres
    }
    parameters={
        'subscribe_depth': False,
        'subscribe_rgb': False,
        'subscribe_scan': False,
        'subscribe_scan_cloud': True,
        'approx_sync': True,
        'Rtabmap/DetectionRate': '1',             # frequency Hz, 0 means go as fast as the data(including laser and image) is coming
        'Rtabmap/ImageBufferSize': '1',           # 0 means process all incoming data
        'Rtabmap/MaxRetrieved': '10',             # TODO
        'Grid/Sensor': '0',                       # 0=laser scan, 1=depth image(s) or 2=both laser scan and depth image(s)
        'Grid/3D': 'false',
        'Grid/CellSize': '0.05',
        'Grid/RayTracing': 'true',
        'Grid/ClusterRadius': '0.1',
        'Grid/MinClusterSize': '10',
        'Grid/RangeMin': '0.5',
        'Grid/RangeMax': '100.0',
        'Grid/NormalsSegmentation': 'false',
        'Grid/MaxGroundHeight': '0.02',           #'-0.01',
        'Grid/MinGroundHeight': '0.0',            #'-0.5',
        'Grid/MaxObstacleHeight': '5.0',
        'Grid/FlatObstacleDetected': 'true',
        'Mem/STMSize': '30',
        'Mem/LaserScanNormalK': '5',              # rich features environment or refraction surface
        'Mem/LaserScanNormalRadius': '0',         # corridor-like, large flat surfaces, and sparse features environment
        'Mem/LaserScanVoxelSize': '0.1',
        'Mem/NotLinkedNodesKept': 'true',         # to suppress the size of db
        'Mem/ReduceGraph': 'true',                # to suppress the size of db
        'Mem/BinDataKept': 'false',               # to suppress the size of db
        'Reg/Strategy': '1',                      # 0=Vis, 1=Icp, 2=VisIcp
        'Reg/Force3DoF': 'true',
        'Icp/VoxelSize': '0.1',
        'Icp/PointToPlane': 'true',
        'Icp/PointToPlaneK': '5',                 # rich features environment or refraction surface
        'Icp/PointToPlaneRadius': '0',          # corridor-like, large flat surfaces, and sparse features environment
        'Icp/PointToPlaneMinComplexity': '0.0125', # 0.02 lower it for corridor-like, large flat surfaces, and sparse features environment
        'Icp/Iterations': '40',
        'Icp/Epsilon': '0.001',
        'Icp/MaxTranslation': '0.8',              # TODO 0.2
        'Icp/MaxCorrespondenceDistance': '0.3',   # TODO 0.1
        'Icp/Strategy': '1',                      # 0=Point Cloud Library, 1=libpointmatcher, 2=CCCoreLib (CloudCompare)
        'Icp/OutlierRatio': '0.8',
        'Icp/CorrespondenceRatio': '0.1',
        'Optimizer/Strategy': '3',                # 0=TORO(i-100), 1=g2o, 2=GTSAM and 3=Ceres(i-20)
        'Optimizer/Iterations': '22',
        'Optimizer/Robust': 'true',
        'Optimizer/GravitySigma': '0',            # Disable imu constraints (we are already in 2D)
        'RGBD/AngularUpdate': '0.05',
        'RGBD/LinearUpdate': '0.05',
        'RGBD/CreateOccupancyGrid': 'true',
        'RGBD/OptimizeFromGraphEnd': 'false',
        'RGBD/OptimizeMaxError': '0',             # should be 0 if Optimizer/Robust is true
        'RGBD/NeighborLinkRefining': 'true',      # Do odometry correction with consecutive laser scans
        'RGBD/ProximityBySpace': 'true',          # Local loop closure detection (using estimated position) with locations in WM
        'RGBD/ProximityByTime': 'false',          # Local loop closure detection with locations in STM
        'RGBD/ProximityPathMaxNeighbors': '0',   # Do also proximity detection by space by merging close scans together.
        'RGBD/ProximityMaxGraphDepth': '0',       # 0 means no limit
        'RGBD/ProximityMaxPaths': '0',            # 0 means no limit
        'RGBD/ProximityOdomGuess': 'true',
        'RGBD/Enabled': 'true',                   # for visual, along with subscribe_rgb=true
        'Vis/MinInliers':'20',                    # for visual
        'Vis/MaxFeatures':'1000',                 # for visual
        'Vis/SSC': 'true',
        'Kp/MaxFeatures': '500',                  # for visual, Maximum features extracted from the images (0 means not bounded, <0 means no extraction)
        'Kp/SSC': 'true',
    }

    if icp_odom.lower() in ("true", "1"): # in case it is a string
        parameters['Icp/Strategy'] = '1'

    remappings_robot_odom = [
        ('scan_cloud', '/rslidar_points'),
        ('odom', '/odometry/filtered'),
    ]
    if rgb.lower() in ("true", "1"): # in case it is a string
        parameters['subscribe_rgb'] = True
        parameters['Reg/Strategy'] = '2'
        remappings_robot_odom.extend([
            ('rgb/camera_info', '/camera_info'),
            ('rgb/image', '/image_raw'),
        ])
    if create_dict.lower() in ("true", "1"): # in case it is a string
        parameters['Mem/NotLinkedNodesKept'] = 'false'
        parameters['Mem/ReduceGraph'] = 'false'
        parameters['Mem/BinDataKept'] = 'true'
        parameters['Vis/SSC'] = 'false'
        parameters['Kp/SSC'] = 'false'
    if fixed_dict.lower() in ("true", "1"): # in case it is a string
        parameters['Kp/IncrementalDictionary'] = 'false'
        parameters['Kp/DictionaryPath'] = '/home/nvidia/.ros/t_02.db'
    
    if incremental.lower() in ("true", "1"): # in case it is a string
        arguments=[]
    else:
        arguments=[
            '-d', # This will delete the previous database (~/.ros/rtabmap.db)
        ]

    start_rtabmap_odom_cmd = Node(
        package='rtabmap_odom', executable='icp_odometry', output='screen',
        parameters=[
            parameters_odom,
            parameters,
            {
                'frame_id': 'base_link',
                'odom_frame_id':'icp_odom',
                'guess_frame_id':'odom',
                "publish_tf": True,
                'wait_for_transform': 0.2,
                'expected_update_rate': 25.0, #15.0,
                'deskewing': False,
                'use_sim_time': use_sim_time,
                'sync_queue_size': 30,
                'topic_queue_size': 30,
            },
        ],
        remappings=[
            ('scan_cloud', '/rslidar_points'),
        ],
        condition=IfCondition(LaunchConfiguration("icp_odom")),
    )
            
    start_rtabmap_util_cmd = Node(
        package='rtabmap_util', executable='point_cloud_assembler', output='screen',
        parameters=[
            {
                'max_clouds': 20,
                'fixed_frame_id': '',
                'use_sim_time': use_sim_time,
                'sync_queue_size': 30,
                'topic_queue_size': 30,
            },
        ],
        remappings=[
            ('cloud', 'odom_filtered_input_scan'),
        ],
        condition=IfCondition(LaunchConfiguration("icp_odom")),
    )

    start_rtabmap_slam_icp_cmd = Node(
        package='rtabmap_slam', executable='rtabmap', output='screen',
        parameters=[
            {
                'frame_id': 'base_link',
                'wait_for_transform': 0.25,
                'use_sim_time': use_sim_time,
            },
            parameters,
        ],
        remappings=[
            ('scan_cloud', 'assembled_cloud'),
        ],
        arguments=arguments,
        condition=IfCondition(LaunchConfiguration("icp_odom")),
    )

    start_rtabmap_slam_cmd = Node(
        package='rtabmap_slam', executable='rtabmap', output='screen',
        parameters=[
            parameters,
            {
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'wait_for_transform': 0.25,
                'queue_size': 50, # increase from default 10
                'use_sim_time': use_sim_time,
            },
        ],
        remappings=remappings_robot_odom,
        arguments=arguments,
        condition=UnlessCondition(LaunchConfiguration("icp_odom")),
    )

    # map server
    start_map_saver_server_cmd = Node(
        package='nav2_map_server',
        executable='map_saver_server',
        output='screen',
        parameters=[
            {'save_map_timeout': 20.0},
            {'free_thresh_default': 0.25},
            {'occupied_thresh_default': 0.65}]
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
        start_rslidar_cmd,
        start_usb_cam_cam,
        start_whi_qrcode_pose_cmd,
        start_whi_landmark_cmd,
        start_rtabmap_odom_cmd,
        start_rtabmap_util_cmd,
        start_rtabmap_slam_icp_cmd,
        start_rtabmap_slam_cmd,
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
            description='top-level namespace'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
            description='use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('autostart', default_value='true',
            description='automatically startup the nav2 stack'),
        DeclareLaunchArgument("vehicle", default_value="L1",
            description="the mobile robot series"),
        DeclareLaunchArgument("vehicle_model", default_value="diff",
            description="the mobile robot's dynamic model"),
        DeclareLaunchArgument('use_ekf', default_value='true',
            description='use ekf to fuse localization'),
        DeclareLaunchArgument('deskewing', default_value='false',
            description='enable lidar deskewing'),
        DeclareLaunchArgument('icp_odom', default_value='false',
            description='whether to use icp odometry'),
        DeclareLaunchArgument('incremental', default_value='false',
            description='whether to map incrementally'),
        DeclareLaunchArgument('bag_file', default_value='/home/nvidia/maps/offline',
            description='input bag file name'),
        DeclareLaunchArgument('landmark', default_value='false',
            description='wether to use landmark'),
        DeclareLaunchArgument('rgb', default_value='false',
            description='wether to use rgb camera for global closure loop'),
        DeclareLaunchArgument('create_dict', default_value='false',
            description='wether to create fixed dictionary'),
        DeclareLaunchArgument('fixed_dict', default_value='false',
            description='wether to use the fixed dictionary'),
        OpaqueFunction(function=launch_setup)
    ])
