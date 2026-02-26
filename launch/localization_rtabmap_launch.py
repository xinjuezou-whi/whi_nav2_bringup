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
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    # Input parameters declaration
    # evaluate substitutions at runtime
    namespace = LaunchConfiguration('namespace')
    db_file = LaunchConfiguration('db_file')
    icp_odom = LaunchConfiguration("icp_odom").perform(context)
    rgb = LaunchConfiguration("rgb").perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    
    parameters={
        # database
        "database_path": LaunchConfiguration('db_file'),
        # rtabmap_ros
        'subscribe_depth': False,
        'subscribe_stereo': False,
        'subscribe_rgb': False,
        'subscribe_scan': False,
        'subscribe_scan_cloud': True,
        'approx_sync': True,
        # rtabmap
        'Rtabmap/DetectionRate': '5',             # frequency Hz, 0 means go as fast as the data(including laser and image) is coming
        'Rtabmap/ImageBufferSize': '1',           # 0 means process all incoming data
        'Rtabmap/MaxRetrieved': '10',
        'Rtabmap/LoopThr': '0.11',
        'Grid/Sensor': '0',                       # 0=laser scan, 1=depth image(s) or 2=both laser scan and depth image(s)
        'Grid/RangeMin': '0.5',
        'Grid/RangeMax': '100.0',
        'Grid/CellSize': '0.1',
        'Grid/MapFrameProjection': 'false',       # from Mathieu
        'Grid/NormalsSegmentation': 'false',      # ************************ trying
        'Grid/MaxObstacleHeight': '5.0',
        'Grid/MinGroundHeight': '0.0',            #'-0.5',
        'Grid/MaxGroundHeight': '0.05',           #'-0.01',
        'Grid/3D': 'false',
        'Grid/RayTracing': 'true',
        # 'GridGlobal/UpdateError': '0.1',          # from Mathieu
        'Mem/NotLinkedNodesKept': 'true',         # to suppress the size of db
        'Mem/STMSize': '100',                     # from Mathieu
        'Mem/ReduceGraph': 'true',                # to suppress the size of db
        'Mem/BinDataKept': 'false',                # to suppress the size of db
        # 'Mem/DepthAsMask': 'false',               # from Mathieu
        # 'Mem/StereoFromMotion': 'true',           # from Mathieu
        'Mem/UseOdomGravity': 'true',             # ************************ trying
        'Reg/Strategy': '1',                      # 0=Vis, 1=Icp, 2=VisIcp
        'Reg/Force3DoF': 'false',                 # ************************ trying
        'Icp/Strategy': '1',                      # 0=Point Cloud Library, 1=libpointmatcher, 2=CCCoreLib (CloudCompare)
        'Icp/MaxTranslation': '5.0',              # ************************ trying
        'Icp/VoxelSize': '0.15',                  # *********************** trying
        'Icp/DownsamplingStep': '1',              # *********************** trying
        'Icp/MaxCorrespondenceDistance': '0.25',  # *********************** trying
        'Icp/Iterations': '50',                   # *********************** trying
        'Icp/Epsilon': '0.0025',                   # *********************** trying
        'Icp/CorrespondenceRatio': '0.1',         # default 0.1, "Ratio of matching correspondences to accept the transform."
        'Icp/Force4DoF': 'false',                  # ************************ trying
        'Icp/PointToPlane': 'true',
        'Icp/PointToPlaneK': '10',                # ************************ trying
        'Icp/PointToPlaneRadius': '0',            # corridor-like, large flat surfaces, and sparse features environment
        'Icp/PointToPlaneGroundNormalsUp': '0.8', # from Mathieu
        'Icp/PointToPlaneMinComplexity': '0.025', # 0.02 higher it for corridor-like, large flat surfaces, and sparse features environment
        'Icp/PointToPlaneLowComplexityStrategy': '1', # from Mathieu
        'Icp/OutlierRatio': '0.85',
        'Icp/PMMatcherKnn': '30',                 # ************************ trying
        'Icp/PMMatcherEpsilon': '0.00001',        # ************************ trying
        'Optimizer/Strategy': '0',                # 0=TORO(i-100), 1=g2o(i-20), 2=GTSAM(i-20) and 3=Ceres(i-20)
        'Optimizer/Iterations': '60',
        'Optimizer/Robust': 'true',               # Robust graph optimization using Vertigo (only work for g2o and GTSAM optimization strategies)."
        'RGBD/AngularUpdate': '0.05',
        'RGBD/LinearUpdate': '0.05',
        'RGBD/ForceOdom3DoF': 'false',            # ************************ trying
        'RGBD/CreateOccupancyGrid': 'true',
        'RGBD/OptimizeMaxError': '0',             # should be 0 if Optimizer/Robust is true
        # 'RGBD/LocalRadius': '20',                 # from Mathieu
        'RGBD/NeighborLinkRefining': 'true',      # Do odometry correction with consecutive laser scans
        'RGBD/LoopClosureReextractFeatures': 'false', # from Mathieu
        'RGBD/ProximityBySpace': 'true',          # Local loop closure detection (using estimated position) with locations in WM
        'RGBD/ProximityMaxGraphDepth': '0',       # default 50, Set 0 to ignore for huge map
        'RGBD/ProximityMaxPaths': '0',            # 0 means no limit
        'RGBD/ProximityPathFilteringRadius': '3.0', # ************************ trying
        'RGBD/ProximityPathMaxNeighbors': '5',    # Maximum neighbor nodes compared on each path for one-to-many proximity detection. Set to 0 to disable one-to-many proximity detection (by merging the laser scans)
        'RGBD/ProximityOdomGuess': 'true',
        'RGBD/Enabled': 'true',                   # for visual, along with subscribe_rgb=true
        'Vis/EstimationType': '2',                # 0:3D->3D, 1:3D->2D (PnP), 2:2D->2D (Epipolar Geometry)
        'Vis/EpipolarGeometryVar': '0.1',         # *********************** trying
        'Vis/MinInliers':'10',                    # *********************** trying
        'Vis/MaxFeatures':'1000',                 # for visual
        'Vis/CorGuessMatchToProjection': 'true',  # ************************ trying
        'Kp/MaxFeatures': '500',                  # for visual, Maximum features extracted from the images (0 means not bounded, <0 means no extraction)
        # odometry
        'Odom/Strategy': '0',                     # 0=Frame-to-Map (F2M) 1=Frame-to-Frame (F2F) 2=Fovis 3=viso2 4=DVO-SLAM 5=ORB_SLAM2 6=OKVIS 7=LOAM 8=MSCKF_VIO 9=VINS-Fusion 10=OpenVINS 11=FLOAM 12=Open3D
        'Odom/Holonomic': 'false',                # ************************ trying
        'Odom/FilteringStrategy': '2',            # ************************ trying
        # 'Odom/ScanKeyFrameThr': '0.8',            # from Mathieu
        # 'OdomF2M/ScanMaxSiz': '15000',            # from Mathieu
        'OdomF2M/ScanSubtractRadius': '0.15',     # Radius used to filter points of a new added scan to local map. This could match the voxel size of the scans
        'OdomF2M/BundleAdjustment': '1',          # Local bundle adjustment: 0=disabled, 1=g2o, 2=cvsba, 3=Ceres
        # localization
        # 'Kp/FlannIndexSaved': 'True',
        # 'RGBD/MaxOdomCacheSize': '0',
        # 'RGBD/ProximityGlobalScanMap': 'True',
        'Mem/IncrementalMemory': 'False',
        'Mem/InitWMWithAllNodes': 'True',
    }

    remappings_rtabmap= [
        ('scan_cloud', 'rslidar_points'),
        # ('imu', 'imu_data'),
    ]

    if rgb.lower() in ("true", "1"): # in case it is a string:
        parameters['subscribe_rgb']=True
        parameters['Reg/Strategy']='2'
        remappings_rtabmap.extend([
            ('rgb/camera_info', 'camera_info'),
            ('rgb/image', 'image_raw'),
        ])

    # with no fuse
    # if icp_odom.lower() in ("true", "1"): # in case it is a string:
    #     remappings_rtabmap.extend([
    #         ('odom', 'icp_odom'),
    #     ])
    # else:
    #     remappings_rtabmap.extend([
    #         ('odom', 'odometry/filtered'),
    #     ])

    # RGB camera
    usb_cam_launch_file = PathJoinSubstitution([
        FindPackageShare('usb_cam'),
        'launch',
        'launch.py'
    ])
    start_usb_cam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(usb_cam_launch_file),
        launch_arguments={
            'namespace': namespace,
        }.items(),
        condition=IfCondition(LaunchConfiguration("rgb")),
    )

    # rtab map
    # ICP odometry (optional)
    start_rtabmap_odom_cmd = Node(
        package='rtabmap_odom', executable='icp_odometry', output='screen',
        parameters=[parameters, 
                    {
                        'use_sim_time': use_sim_time,
                        # 'odom_frame_id':'icp_odom', # with no fuse
                        # 'guess_frame_id':'odom', # with no fuse
                        'odom_frame_id':'odom', # with fuse
                        "publish_tf": False, # with fuse
                    }
        ],
        remappings=remappings_rtabmap + [
            ('odom', 'icp_odom'),
        ],
        condition=IfCondition(LaunchConfiguration("icp_odom")),
    )

    # obstacles_detection
    start_rtabmap_obstacle_detection_cmd = Node(
        namespace=namespace,
        package='rtabmap_util', executable='obstacles_detection', output='screen',
        parameters=[
            {
                'frame_id': 'base_link',
                'map_frame_id': 'map',
                'wait_for_transform': 0.5,
                'Grid/Sensor': '0',                       # 0=laser scan, 1=depth image(s) or 2=both laser scan and depth image(s)
                'Grid/RangeMin': '0.0',
                'Grid/RangeMax': '100.0',
                'Grid/CellSize': '0.15',                  # ************************ suggested
                'Grid/NormalsSegmentation': 'true',
                'Grid/MaxObstacleHeight': '5.0',
                'Grid/3D': 'false',
                'Grid/RayTracing': 'true',
            }
        ],
        remappings=[
            ('cloud', 'rslidar_points'),
        ],
    )
    
    # SLAM
    start_rtabmap_slam_cmd = Node(
        namespace=namespace,
        package='rtabmap_slam', executable='rtabmap', output='screen',
        parameters=[
            parameters,
            {
                'use_sim_time': use_sim_time,
                'frame_id': 'base_link',
                'map_frame_id': 'map',
                'odom_frame_id': 'odom',
                'wait_for_transform': 0.25,
                'topic_queue_size': 50, # increase from default 10,
                'sync_queue_size': 50, # increase from default 10
            }
        ],
        remappings=remappings_rtabmap + [
            ('odom', 'odometry/filtered'), # with fuse
        ],
    )

    launch_nodes = [
        start_usb_cam_cmd,
        start_rtabmap_odom_cmd,
        start_rtabmap_obstacle_detection_cmd,
        start_rtabmap_slam_cmd,
    ]

    return launch_nodes

def generate_launch_description():
    return LaunchDescription([
        # Declare the launch arguments
        DeclareLaunchArgument('namespace', default_value='',
            description='Top-level namespace'),
        DeclareLaunchArgument('db_file', default_value='/home/nvidia/.ros/rtabmap.db',
            description='database file name'),
        DeclareLaunchArgument('icp_odom', default_value='false',
            description='Whether to use icp odom'),
        DeclareLaunchArgument('rgb', default_value='false',
            description='Whether to use RGB visual'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('params_file', default_value='',
            description='Full path to the ROS2 parameters file to use for all launched nodes'),
        DeclareLaunchArgument('autostart', default_value='true',
            description='Automatically startup the nav2 stack'),
        DeclareLaunchArgument('use_composition', default_value='False',
            description='Use composed bringup if True'),
        DeclareLaunchArgument('container_name', default_value='nav2_container',
            description='the name of conatiner that nodes will load in if use composition'),
        DeclareLaunchArgument('use_respawn', default_value='False',
            description='Whether to respawn if a node crashes. Applied when composition is disabled'),
        DeclareLaunchArgument('log_level', default_value='info',
            description='log level'),
        OpaqueFunction(function=launch_setup)
    ])
