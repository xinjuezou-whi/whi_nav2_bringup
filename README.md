# whi_nav2_bringup
Navigation stack under ROS 2

## Dependency
```
sudo apt install ros-$ROS_DISTRO-robot-localization
```

## Mapping

- 2D with **Slam Toolbox**
  | Argument | Default | Function | Requirement |
  |----------|---------|----------|-------------|
  | vehicle  | L1 | AMR series: L1, L2, E1, E2 | required |
  | vehicle_model | diff | ARM kinematics: diff, diff4, swerve | required |
  | use_ekf | true | fuse the sources of odometry and smoothing by Extended Kalman Filter | optional |
  | bag_file | /home/whi/maps/offline | the file path of record bag data for offline mapping. combined with use_sim_time = true | optional |

  Example of online mapping
  ```
  # default use_ekf:=true, bag_file:=''
  ros2 launch whi_nav2_bringup mapping_slam_toolbox.py vehicle:=E1 vehicle_model:=swerve
  ```

  Example of offline mapping
  ```
  # default use_ekf:=true
  ros2 launch whi_nav2_bringup mapping_slam_toolbox.py vehicle:=E1 vehicle_model:=swerve use_sim_time:=true bag_file:=/home/whi/maps/offline
  ```

  **Save the map**

  In `Slam Toolbox Plugin` tab, input the absolute path of map, click `Save Map` button to save the `*.pgm` and `*.yaml` map, which can be viewed as an image, and will be used by localization of `amcl`, or used for editing the keep-out zone:
  <p>
    <img width="1845" height="1051" alt="image" src="https://github.com/user-attachments/assets/6a082d84-152b-4534-ae3e-d2edf9eab36f" />
  </p>

  In `Slam Toolbox Plugin` tab, input the absolute path of map, click `Serialize Map` button to save the `*.data` and `*.posegraph` map, which are used by localization of slam toolbox:
  <p>
    <img width="1851" height="1057" alt="image" src="https://github.com/user-attachments/assets/7213b14d-6e29-438b-b096-124887d0520c" />
  </p>

  <br>
- 2D or 3D with **Cartographer**
  | Argument | Default | Function | Requirement |
  |----------|---------|----------|-------------|
  | vehicle  | L1 | AMR series: L1, L2, E1, E2 | required |
  | vehicle_model | diff | ARM kinematics: diff, diff4, swerve | required |
  | use_ekf | true | fuse the sources of odometry and smoothing by Extended Kalman Filter | optional |
  | use_3d | false | the flag of toggling 2D and 3D mode | optional |

  Example of 2D mapping
  ```
  # default use_ekf:=true, use_3d:=false
  ros2 launch whi_nav2_bringup mapping_cartographer.py vehicle:=E1 vehicle_model:=swerve
  ```

  Example of 3D mapping
  ```
  # default use_ekf:=true
  ros2 launch whi_nav2_bringup mapping_cartographer.py vehicle:=E1 vehicle_model:=swerve use_3d:=true
  ```

  <p>
    <img width="2281" height="1300" alt="image" src="https://github.com/user-attachments/assets/82935680-90e9-4705-81b6-7c2608c92363" />
  </p>

  **Save the map**

  Open a new terminal, run bellowing two commands to save both image map and map data. The first will save the `*.pgm` and `*.yaml` map, which can be viewed as an image; And the second one will save the `*.pbstream` map data, which are used by localization of Cartographer:
  ```
  # image map
  ros2 run nav2_map_server map_saver_cli -f /home/whi/maps/my_map2d
  # map data
  ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '/home/whi/maps/my_map.pbstream'}"
  ```

  <br>
- 3D with **Rtabmap**
  | Argument | Default | Function | Requirement |
  |----------|---------|----------|-------------|
  | vehicle  | L1 | AMR series: L1, L2, E1, E2 | required |
  | vehicle_model | diff | ARM kinematics: diff, diff4, swerve | required |
  | use_ekf | true | fuse the sources of odometry and smoothing by Extended Kalman Filter | optional |
  | icp_odom | false | use the odometry of ICP - Iterative Closest Point | optional |
  | fast_lio_odom | false | use the odometry of FAST_LIO | optional |
  | db_file | /home/whi/.ros/rtabmap.db | the path of map data | optional |
  | landmark | false | use landmark for global loop closure | optional |
  | rgb | false | use RGB images for global loop closure | optional |

  Example of mapping with EKF odometry and RGB global closure loop
  ```
  # default use_ekf:=true
  ros2 launch whi_nav2_bringup mapping_rtabmap.py vehicle:=E1 vehicle_model:=swerve rgb:=true db_file:=/home/whi/maps/rtabmap.db
  ```

  Example of mapping with FAST_LIO odometry
  ```
  # default rgb:=false
  ros2 launch whi_nav2_bringup mapping_rtabmap.py vehicle:=E1 vehicle_model:=swerve fast_lio_odom:=true db_file:=/home/whi/maps/rtabmap.db
  ```

  <p>
    <img width="2463" height="1404" alt="image" src="https://github.com/user-attachments/assets/b64bd843-6e7e-4f62-afc4-52182d4f02af" />
  </p>

  **Save the map**

  Rtabmap will save the map data automatically at termination

## Navigation

- 2D with **AMCL**
  | Argument | Default | Function | Requirement |
  |----------|---------|----------|-------------|
  | vehicle  | L1 | AMR series: L1, L2, E1, E2 | required |
  | vehicle_model | diff | ARM kinematics: diff, diff4, swerve | required |
  | use_ekf | true | fuse the sources of odometry and smoothing by Extended Kalman Filter | optional |
  | map | /home/whi/maps/field_test.yaml | the file path of map | required |

  Example of navigation with EKF odometry and AMCL localization
  ```
  # default use_ekf:=true
  ros2 launch whi_nav2_bringup navigation_local.py vehicle:=E1 vehicle_model:=swerve map:=/home/whi/maps/debug.yaml
  ```

  <br>
- 2D with **Slam Toolbox**
  | Argument | Default | Function | Requirement |
  |----------|---------|----------|-------------|
  | vehicle  | L1 | AMR series: L1, L2, E1, E2 | required |
  | vehicle_model | diff | ARM kinematics: diff, diff4, swerve | required |
  | use_ekf | true | fuse the sources of odometry and smoothing by Extended Kalman Filter | optional |
  | slam_toolbox_map | empty | the file path of map data | required |

  Example of navigation with EKF odometry and Slam Toolbox localization
  ```
  # default use_ekf:=true
  ros2 launch whi_nav2_bringup navigation_local.py vehicle:=E1 vehicle_model:=swerve slam_toolbox_map:=/home/whi/maps/debug
  ```
  > NOTE: the slam toolbox's map file requires none file extension

  <br>
- 2D or 3D with **Cartographer**
  | Argument | Default | Function | Requirement |
  |----------|---------|----------|-------------|
  | vehicle  | L1 | AMR series: L1, L2, E1, E2 | required |
  | vehicle_model | diff | ARM kinematics: diff, diff4, swerve | required |
  | use_ekf | true | fuse the sources of odometry and smoothing by Extended Kalman Filter | optional |
  | cartographer_map | empty | the file path of map data | required |

  Example of navigation with EKF odometry and Cartographer localization
  ```
  # default use_ekf:=true
  ros2 launch whi_nav2_bringup navigation_local.py vehicle:=E1 vehicle_model:=swerve cartographer_map:=/home/whi/maps/debug_carto.pbstream map:=/home/whi/maps/debug_carto.yaml
  ```

  <br>
- 3D with Rtabmap
  | Argument | Default | Function | Requirement |
  |----------|---------|----------|-------------|
  | vehicle  | L1 | AMR series: L1, L2, E1, E2 | required |
  | vehicle_model | diff | ARM kinematics: diff, diff4, swerve | required |
  | use_ekf | true | fuse the sources of odometry and smoothing by Extended Kalman Filter | optional |
  | icp_odom | false | use the odometry of ICP - Iterative Closest Point | optional |
  | fast_lio_odom | false | use the odometry of FAST_LIO | optional |
  | rtabmap_map | empty | the file path of map data | required |

  Example of navigation with EKF odometry and Rtabmap localization with RGB global closure loop
  ```
  # default use_ekf:=true
  ros2 launch whi_nav2_bringup navigation_local.py vehicle:=E1 vehicle_model:=swerve rgb:=true rtabmap_map:=/home/whi/maps/rtabmap.db
  ```

  Example of navigation with FAST_LIO odometry and Rtabmap localization with RGB global closure loop
  ```
  ros2 launch whi_nav2_bringup navigation_local.py vehicle:=E1 vehicle_model:=swerve fast_lio_odom:=true rgb:=true rtabmap_map:=/home/whi/maps/rtabmap_lio.db
  ```
  
### Navigation with fixed route and keepout zone filter

The navigation system supports:
- **Fixed-route navigation**: Follow predefined routes generated from a route graph (`graph_file`).
- **Keepout-zone filtering**: Prevent the robot from entering restricted areas defined by a keepout mask (`keepout_mask_file`).

Both features are optional and can be enabled through launch parameters

- route
  | Argument | Default | Function | Requirement |
  |----------|---------|----------|-------------|
  | graph_file  | empty | the route graph path | optional |

  Example of navigation with EKF odometry and Rtabmap localization with RGB global closure loop
  ```
  # default use_ekf:=true
  ros2 launch whi_nav2_bringup navigation_local.py vehicle:=E1 vehicle_model:=swerve rgb:=true rtabmap_map:=/home/whi/.ros/debug.db graph_file:=/home/whi/maps/route.geojson
  ```
  <p>
    <img width="2464" height="1401" alt="image" src="https://github.com/user-attachments/assets/edd76a5f-6be6-4bae-86ea-f4ebc31e53d6" />
  </p>

  <br>
- keepout zone filter
  | Argument | Default | Function | Requirement |
  |----------|---------|----------|-------------|
  | keepout_mask_file  | empty | the keepout zone mask path | optional |

  Example of navigation with FAST_LIO odometry and Rtabmap localization without RGB global closure loop
  ```
  # default rgb:=false
  ros2 launch whi_nav2_bringup navigation_local.py vehicle:=E1 vehicle_model:=swerve fast_lio_odom:=true rtabmap_map:=/home/whi/.ros/debug.db keepout_mask_file:=/home/whi/maps/debug_keepout.yaml 
  ```

  <p>
    <img width="2469" height="1399" alt="image" src="https://github.com/user-attachments/assets/7c474a91-2f1d-4f1c-b2e8-61ea4fb7979b" />
  </p>
