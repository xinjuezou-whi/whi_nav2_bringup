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

  **Save the map**

  Rtabmap will save the map data automatically at termination

## Navigation

