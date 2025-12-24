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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml

def launch_setup(context, *args, **kwargs):
    # Input parameters declaration
    # evaluate substitutions at runtime
    namespace = LaunchConfiguration('namespace')
    map_yaml_file = LaunchConfiguration('map')
    db_file = LaunchConfiguration('db_file')
    icp_odom = LaunchConfiguration("icp_odom").perform(context)
    rgb = LaunchConfiguration("rgb").perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
    ]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)
    
    parameters={
        'subscribe_depth': False,
        'subscribe_stereo': False,
        'subscribe_rgb': False,
        'subscribe_scan': False,
        'subscribe_scan_cloud': True,
        'approx_sync': True,
        'Rtabmap/DetectionRate': '1',             # frequency Hz, 0 means go as fast as the data(including laser and image) is coming
        'Rtabmap/ImageBufferSize': '1',           # 0 means process all incoming data
        'Rtabmap/MaxRetrieved': '10',
        'Rtabmap/LoopThr': '0.11',
        'Grid/Sensor': '0',                       # 0=laser scan, 1=depth image(s) or 2=both laser scan and depth image(s)
        'Grid/RangeMin': '0.5',
        'Grid/RangeMax': '100.0',
        'Grid/CellSize': '0.05',
        'Grid/MapFrameProjection': 'true',
        'Grid/NormalsSegmentation': 'false',      # ************************ trying
        'Grid/MaxObstacleHeight': '5.0',
        'Grid/MinGroundHeight': '0.0',            #'-0.5',
        'Grid/MaxGroundHeight': '0.05',           #'-0.01',
        'Grid/3D': 'false',
        'Grid/RayTracing': 'true',
        'Mem/NotLinkedNodesKept': 'true',         # to suppress the size of db
        'Mem/ReduceGraph': 'true',                # to suppress the size of db
        'Mem/BinDataKept': 'false',               # to suppress the size of db
        'Mem/UseOdomGravity': 'true',             # ************************ trying
        'Reg/Strategy': '1',                      # 0=Vis, 1=Icp, 2=VisIcp
        'Reg/Force3DoF': 'false',                 # ************************ trying
        'Icp/Strategy': '1',                      # 0=Point Cloud Library, 1=libpointmatcher, 2=CCCoreLib (CloudCompare)
        'Icp/MaxTranslation': '5.0',              # ************************ trying
        'Icp/VoxelSize': '0.12',                  # *********************** trying
        'Icp/DownsamplingStep': '1',              # *********************** trying
        'Icp/MaxCorrespondenceDistance': '0.25',   # *********************** trying
        'Icp/Iterations': '50',                   # ************************ trying
        'Icp/Epsilon': '0.00001',
        'Icp/CorrespondenceRatio': '0.1',         # default 0.1, "Ratio of matching correspondences to accept the transform."
        'Icp/Force4DoF': 'false',                  # ************************ trying
        'Icp/PointToPlane': 'true',
        'Icp/PointToPlaneK': '10',                # ************************ trying
        'Icp/PointToPlaneRadius': '0',            # corridor-like, large flat surfaces, and sparse features environment
        'Icp/PointToPlaneGroundNormalsUp': '0.5', # ************************ trying
        'Icp/PointToPlaneMinComplexity': '0.025', # 0.02 higher it for corridor-like, large flat surfaces, and sparse features environment
        'Icp/PointToPlaneLowComplexityStrategy': '2', # ************************ trying
        'Icp/OutlierRatio': '0.85',
        'Icp/PMMatcherKnn': '30',                 # ************************ trying
        'Icp/PMMatcherEpsilon': '0.00001',        # ************************ trying
        'Optimizer/Strategy': '1',                # 0=TORO(i-100), 1=g2o(i-20), 2=GTSAM(i-20) and 3=Ceres(i-20)
        'Optimizer/Iterations': '30',
        'Optimizer/Robust': 'true',               # Robust graph optimization using Vertigo (only work for g2o and GTSAM optimization strategies)."
        'RGBD/AngularUpdate': '0.05',
        'RGBD/LinearUpdate': '0.05',
        'RGBD/ForceOdom3DoF': 'false',            # ************************ trying
        'RGBD/CreateOccupancyGrid': 'true',
        'RGBD/OptimizeFromGraphEnd': 'false',
        'RGBD/OptimizeMaxError': '0',             # should be 0 if Optimizer/Robust is true
        'RGBD/NeighborLinkRefining': 'true',      # Do odometry correction with consecutive laser scans
        'RGBD/LoopClosureReextractFeatures': 'true',
        'RGBD/ProximityByTime': 'false',          # Local loop closure detection with locations in STM
        'RGBD/ProximityBySpace': 'true',          # Local loop closure detection (using estimated position) with locations in WM
        'RGBD/ProximityMaxGraphDepth': '0',       # default 50, Set 0 to ignore for huge map
        'RGBD/ProximityMaxPaths': '0',            # 0 means no limit
        'RGBD/ProximityPathFilteringRadius': '3.0', # ************************ trying
        'RGBD/ProximityPathMaxNeighbors': '4',    # Maximum neighbor nodes compared on each path for one-to-many proximity detection. Set to 0 to disable one-to-many proximity detection (by merging the laser scans)
        'RGBD/ProximityOdomGuess': 'true',
        'RGBD/Enabled': 'true',                   # for visual, along with subscribe_rgb=true
        'Vis/EstimationType': '2',                # 0:3D->3D, 1:3D->2D (PnP), 2:2D->2D (Epipolar Geometry)
        'Vis/EpipolarGeometryVar': '0.1',         # default 0.1
        'Vis/MinInliers':'10',                    # default 20
        'Vis/MaxFeatures':'1000',                 # for visual
        'Vis/SSC': 'false',
        'Vis/CorGuessMatchToProjection': 'true',  # ************************ trying
        'Kp/SSC': 'false',
        'Kp/MaxFeatures': '500',                  # for visual, Maximum features extracted from the images (0 means not bounded, <0 means no extraction)
        # odometry
        'Odom/Strategy': '0',                     # 0=Frame-to-Map (F2M) 1=Frame-to-Frame (F2F) 2=Fovis 3=viso2 4=DVO-SLAM 5=ORB_SLAM2 6=OKVIS 7=LOAM 8=MSCKF_VIO 9=VINS-Fusion 10=OpenVINS 11=FLOAM 12=Open3D
        'Odom/Holonomic': 'false',                # ************************ trying
        'Odom/GuessMotion': 'true',
        'OdomF2M/ScanSubtractRadius': '0.05',     # Radius used to filter points of a new added scan to local map. This could match the voxel size of the scans
        'OdomF2M/BundleAdjustment': '1',          # Local bundle adjustment: 0=disabled, 1=g2o, 2=cvsba, 3=Ceres
        # localization
        'Kp/FlannIndexSaved': 'True',
        'RGBD/MaxOdomCacheSize': '30',
        'RGBD/ProximityGlobalScanMap': 'True',
        'Mem/IncrementalMemory': 'False',
        'Mem/InitWMWithAllNodes': 'True',
    }

    remappings_rtabmap= [
        ('scan_cloud', '/rslidar_points'),
        ('imu', '/imu_data'),
    ]


    if rgb.lower() in ("true", "1"): # in case it is a string:
        parameters['subscribe_rgb']=True
        parameters['Reg/Strategy']='2'
        remappings_rtabmap.extend([
            ('rgb/camera_info', '/camera_info'),
            ('rgb/image', '/image_raw'),
        ])

    if icp_odom.lower() in ("true", "1") and rgb.lower() in ("false", "1"): # in case it is a string:
        remappings_rtabmap.extend([
            ('odom', 'icp_odom'),
        ])
    else:
        remappings_rtabmap.extend([
            ('odom', '/odometry/filtered'),
        ])

    # RGB camera
    start_usb_cam_cmd = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[
            '/home/nvidia/ros2_ws/src/usb_cam/config/params_hik.yaml'
        ],
        condition=IfCondition(
            PythonExpression([
                "'", rgb, "' == 'true' and '", icp_odom, "' == 'false'"
            ])
        ),
    )

    # rtab map
    # ICP odometry (optional)
    start_rtabmap_odom_cmd = Node(
        condition=IfCondition(
            PythonExpression([
                "'", icp_odom, "' == 'true' and '", rgb, "' == 'false'"
            ])
        ),
        package='rtabmap_odom', executable='icp_odometry', output='screen',
        parameters=[parameters, 
                    {
                        'use_sim_time': use_sim_time,
                        # 'odom_frame_id':'icp_odom',
                        # 'guess_frame_id':'odom',
                        'odom_frame_id':'odom',
                        "publish_tf": False,
                    }
        ],
        remappings=remappings_rtabmap,
    )
    
    # SLAM:
    start_rtabmap_slam_cmd = Node(
        package='rtabmap_slam', executable='rtabmap', output='screen',
        parameters=[
            parameters,
            {
                'use_sim_time': use_sim_time,
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
            }
        ],
        remappings=remappings_rtabmap,
        arguments=[
            # '-d', # This will delete the previous database (~/.ros/rtabmap.db)
            # '--database_path', db_file,
        ],
    )

    start_map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings
    )

    lifecycle_nodes = ['map_server']
    start_lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes}]
    )

    launch_nodes = [
        start_map_server_cmd,
        start_lifecycle_manager_cmd,
        start_usb_cam_cmd,
        start_rtabmap_odom_cmd,
        start_rtabmap_slam_cmd,
    ]

    return launch_nodes

def generate_launch_description():
    return LaunchDescription([
        # Declare the launch arguments
        DeclareLaunchArgument('namespace', default_value='',
            description='Top-level namespace'),
        DeclareLaunchArgument('map',
            description='Full path to map yaml file to load'),
        DeclareLaunchArgument('db_file', default_value='home/nvidia/.ros/rtabmap.db',
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
