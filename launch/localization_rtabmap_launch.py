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
    icp_odom = LaunchConfiguration("icp_odom")
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
        'frame_id': 'base_link',
        'use_sim_time': use_sim_time,
        'subscribe_depth': False,
        'subscribe_rgb': False,
        'subscribe_scan': False,
        'subscribe_scan_cloud': True,
        'approx_sync': True,
        'use_action_for_goal': True,
        'Reg/Strategy': '1',                    # 0=Vis, 1=Icp, 2=VisIcp
        'Reg/Force3DoF': 'false',
        'Grid/Sensor': '0',                     # 0=laser scan, 1=depth image(s) or 2=both laser scan and depth image(s)
        'Grid/RayTracing': 'false',
        'Grid/RangeMin': '0.5', # ignore laser scan points on the robot itself
        'Grid/RangeMax': '60.0',
        'Grid/NormalsSegmentation': 'false',
        'Grid/MaxGroundHeight': '0.03',
        'Grid/MinGroundHeight': '0.0',
        'Grid/MaxObstacleHeight': '2.0',
        'Grid/FlatObstacleDetected': 'true',
        'Icp/VoxelSize': '0.1',
        'Icp/PointToPlaneK': '0',
        'Icp/PointToPlaneRadius': '0.5',
        'Icp/PointToPlane': 'true',
        'Icp/Iterations': '40',
        'Icp/Epsilon': '0.001',
        'Icp/MaxTranslation': '2',
        'Icp/MaxCorrespondenceDistance': '0.5',
        'Icp/Strategy': '1',                    # 0=Point Cloud Library, 1=libpointmatcher, 2=CCCoreLib (CloudCompare)
        'Icp/OutlierRatio': '0.8',
        'Icp/CorrespondenceRatio': '0.2',
        'Optimizer/Strategy': '3',              # 0=TORO, 1=g2o, 2=GTSAM and 3=Ceres
        'Optimizer/Robust': 'true',
        'Optimizer/Iterations': '30',
        'Optimizer/GravitySigma': '0',         # Disable imu constraints (we are already in 2D)
        'RGBD/CreateOccupancyGrid': 'false',
        'RGBD/OptimizeMaxError': '0',          # should be 0 if Optimizer/Robust is true
        'RGBD/NeighborLinkRefining': 'true',   # Do odometry correction with consecutive laser scans
        'RGBD/ProximityBySpace': 'true',       # Local loop closure detection (using estimated position) with locations in WM
        'RGBD/ProximityByTime': 'false',       # Local loop closure detection with locations in STM
        'RGBD/ProximityPathMaxNeighbors': '10', # Do also proximity detection by space by merging close scans together.
        'RGBD/ProximityMaxGraphDepth': '0',    # 0 means no limit
        'RGBD/ProximityOdomGuess': 'true',
        # localization
        'Mem/IncrementalMemory': 'False',
        'Mem/InitWMWithAllNodes': 'True',
    }

    remappings_rtabmap_slam = [
        ('scan_cloud', '/rslidar_points'),
        # ('odom', 'icp_odom'),
        ('odom', '/odometry/filtered'),
    ]
    parameters_rtabmap = parameters

    if rgb.lower() in ("true", "1"): # in case it is a string:
        parameters_rtabmap['subscribe_rgb']=True
        parameters_rtabmap['Reg/Strategy']='2'
        remappings_rtabmap_slam.extend([
            ('rgb/camera_info', '/camera_info'),
            ('rgb/image', '/image_raw'),
        ])

    # RGB camera
    start_usb_cam_cmd = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[
            '/home/nvidia/ros2_humble/src/usb_cam/config/params_hik.yaml'
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
                        # 'odom_frame_id':'icp_odom',
                        # 'guess_frame_id':'odom',
                        'odom_frame_id':'odom',
                        "publish_tf": False,
                    }
        ],
        remappings=[
            ('scan_cloud', '/rslidar_points'),
            ('odom', 'icp_odom'),
        ],
    )
    
    # SLAM:
    start_rtabmap_slam_cmd = Node(
        package='rtabmap_slam', executable='rtabmap', output='screen',
        parameters=[parameters_rtabmap],
        remappings=remappings_rtabmap_slam,
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
