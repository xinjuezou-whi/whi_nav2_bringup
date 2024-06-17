#!/usr/bin/env python3
# Copyright 2026 WheelHub Intelligent
# RTAB-Map Multi-Floor Database Switcher Node

import os
import time
import yaml
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from std_srvs.srv import SetBool
from rtabmap_msgs.srv import LoadDatabase
from nav2_msgs.srv import ClearEntireCostmap


class RtabmapDbSwitcherNode(Node):

    def __init__(self):
        super().__init__('rtabmap_db_switcher_node')

        # Reentrant Callback Group to prevent deadlocks during nested service calls
        self.cb_group = ReentrantCallbackGroup()

        # Parameters
        self.declare_parameter(
            'config_file',
            '/home/nvidia/ros2_ws/src/whi_nav2_bringup/config/floor_config.yaml'
        )
        self.declare_parameter('default_floor', 'Floor_1')

        self.current_floor = self.get_parameter('default_floor').get_parameter_value().string_value
        config_path = self.get_parameter('config_file').get_parameter_value().string_value

        self.floor_db_map = {}
        self._load_config(config_path)

        self.last_map_stamp = None

        # QoS for Floor Topic (Transient Local / Latched)
        latch_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        # Topic Interfaces
        self.floor_pub = self.create_publisher(String, 'current_floor', latch_qos)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._map_callback, 10, callback_group=self.cb_group
        )

        # Service exposed to Multi-Floor Dispatcher
        self.switch_floor_srv = self.create_service(
            SetBool, 'switch_floor_db', self.handle_switch_floor_request, callback_group=self.cb_group
        )

        # External Service Clients
        self.rtabmap_load_db_cli = self.create_client(
            LoadDatabase, '/rtabmap/load_database', callback_group=self.cb_group
        )
        self.clear_global_costmap_cli = self.create_client(
            ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap', callback_group=self.cb_group
        )
        self.clear_local_costmap_cli = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap', callback_group=self.cb_group
        )

        # One-shot startup timer to execute initial database check/load for default floor
        self.init_timer = self.create_timer(0.5, self._initial_floor_setup, callback_group=self.cb_group)

        self.get_logger().info(f"🚀 RTAB-Map DB Switcher Ready. Configured default floor: [{self.current_floor}]")

    def _initial_floor_setup(self):
        """Timer callback that triggers initial map database check once on startup."""
        self.destroy_timer(self.init_timer)
        self.get_logger().info(f"⚡ Checking map database state for requested floor: [{self.current_floor}]")
        self.switch_to_floor_by_name(self.current_floor)

    def _load_config(self, config_path: str):
        """Loads floor database paths dynamically from floor_config.yaml."""
        if not os.path.exists(config_path):
            self.get_logger().error(f"Config file not found at '{config_path}'!")
            return

        with open(config_path, 'r') as f:
            data = yaml.safe_load(f)
            floors = data.get('floors', {})
            for floor_name, details in floors.items():
                if 'db_path' in details:
                    self.floor_db_map[floor_name] = os.path.expanduser(details['db_path'])

        self.get_logger().info(f"Loaded Floor Database Mapping: {self.floor_db_map}")

    def _map_callback(self, msg: OccupancyGrid):
        """Tracks the timestamp of incoming /map occupancy grid messages."""
        self.last_map_stamp = msg.header.stamp

    def handle_switch_floor_request(self, request, response):
        """Service callback to handle floor switch requests."""
        target_floor = "Floor_2" if request.data else "Floor_1"
        success = self.switch_to_floor_by_name(target_floor)
        response.success = success
        response.message = f"Switched active map to {target_floor}" if success else "Switch failed"
        return response

    def switch_to_floor_by_name(self, target_floor_name: str) -> bool:
        """Executes database load if needed, waits for map reload, and flushes costmaps."""
        if target_floor_name not in self.floor_db_map:
            self.get_logger().error(f"Target floor '{target_floor_name}' not found in floor_config.yaml!")
            return False

        # --- PRE-CHECK: Skip reload if already on target floor ---
        if target_floor_name == self.current_floor:
            self.get_logger().info(f"ℹ️ System is already active on [{target_floor_name}]. Skipping database reload.")
            # Re-publish current floor to guarantee latched topic consistency
            msg = String()
            msg.data = self.current_floor
            self.floor_pub.publish(msg)
            return True

        db_path = self.floor_db_map[target_floor_name]
        if not os.path.exists(db_path):
            self.get_logger().error(f"Database path '{db_path}' does not exist on disk!")
            return False

        self.get_logger().info(f"⏳ Switching database to '{target_floor_name}' ({db_path})...")

        prev_stamp = self.last_map_stamp

        # 1. Send load database service request
        if not self._call_rtabmap_load_db_sync(db_path):
            self.get_logger().error("Failed during /rtabmap/load_database execution!")
            return False

        # 2. Wait up to 10s for RTAB-Map to broadcast a fresh OccupancyGrid on /map
        self.get_logger().info("Waiting for fresh /map occupancy grid from RTAB-Map...")
        start_wait = time.time()
        map_updated = False

        while time.time() - start_wait < 10.0:
            if self.last_map_stamp != prev_stamp and self.last_map_stamp is not None:
                map_updated = True
                self.get_logger().info("Received fresh occupancy grid from RTAB-Map.")
                break
            time.sleep(0.2)

        if not map_updated:
            self.get_logger().warn("Timed out waiting for new /map topic update. Proceeding with costmap clear...")

        # 3. Clear Nav2 costmaps
        self._clear_nav2_costmaps()
        time.sleep(0.5)

        # 4. Update current floor state & publish topic
        self.current_floor = target_floor_name
        msg = String()
        msg.data = self.current_floor
        self.floor_pub.publish(msg)

        self.get_logger().info(f"🎉 Floor switch to [{self.current_floor}] successfully finished!")
        return True

    def _call_rtabmap_load_db_sync(self, db_path: str) -> bool:
        """Sends synchronous load_database request to RTAB-Map."""
        if not self.rtabmap_load_db_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service /rtabmap/load_database is unavailable!")
            return False

        req = LoadDatabase.Request()
        req.database_path = db_path
        req.clear = False

        future = self.rtabmap_load_db_cli.call_async(req)
        
        while not future.done():
            time.sleep(0.1)

        res = future.result()
        return res is not None

    def _clear_nav2_costmaps(self):
        """Clears Nav2 global and local costmaps."""
        req = ClearEntireCostmap.Request()
        if self.clear_global_costmap_cli.wait_for_service(timeout_sec=2.0):
            self.clear_global_costmap_cli.call_async(req)
        if self.clear_local_costmap_cli.wait_for_service(timeout_sec=2.0):
            self.clear_local_costmap_cli.call_async(req)
        self.get_logger().info("Nav2 global and local costmaps cleared.")


def main(args=None):
    rclpy.init(args=args)
    node = RtabmapDbSwitcherNode()

    # MultiThreadedExecutor is required so background callbacks run during blocking loops
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()















###################################################################################################
# import os
# import time
# import yaml
# import rclpy
# from rclpy.node import Node
# from rclpy.callback_groups import ReentrantCallbackGroup
# from rclpy.executors import MultiThreadedExecutor
# from rclpy.qos import (
#     QoSProfile,
#     QoSDurabilityPolicy,
#     QoSReliabilityPolicy,
#     QoSHistoryPolicy,
# )

# from nav_msgs.msg import OccupancyGrid
# from std_msgs.msg import String
# from std_srvs.srv import SetBool
# from rtabmap_msgs.srv import LoadDatabase
# from nav2_msgs.srv import ClearEntireCostmap


# class RtabmapDbSwitcherNode(Node):

#     def __init__(self):
#         super().__init__('rtabmap_db_switcher_node')

#         # Reentrant Callback Group to prevent deadlocks during nested service calls
#         self.cb_group = ReentrantCallbackGroup()

#         # Parameters
#         self.declare_parameter(
#             'config_file',
#             '/home/nvidia/ros2_ws/src/whi_nav2_bringup/config/floor_config.yaml'
#         )
#         self.declare_parameter('default_floor', 'Floor_1')

#         self.current_floor = self.get_parameter('default_floor').get_parameter_value().string_value
#         config_path = self.get_parameter('config_file').get_parameter_value().string_value

#         self.floor_db_map = {}
#         self._load_config(config_path)

#         self.last_map_stamp = None

#         # QoS for Floor Topic (Transient Local / Latched)
#         latch_qos = QoSProfile(
#             depth=1,
#             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
#             reliability=QoSReliabilityPolicy.RELIABLE,
#             history=QoSHistoryPolicy.KEEP_LAST,
#         )

#         # Topic Interfaces
#         self.floor_pub = self.create_publisher(String, 'current_floor', latch_qos)
#         self.map_sub = self.create_subscription(
#             OccupancyGrid, '/map', self._map_callback, 10, callback_group=self.cb_group
#         )

#         # Service exposed to Multi-Floor Dispatcher
#         self.switch_floor_srv = self.create_service(
#             SetBool, 'switch_floor_db', self.handle_switch_floor_request, callback_group=self.cb_group
#         )

#         # External Service Clients
#         self.rtabmap_load_db_cli = self.create_client(
#             LoadDatabase, '/rtabmap/load_database', callback_group=self.cb_group
#         )
#         self.clear_global_costmap_cli = self.create_client(
#             ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap', callback_group=self.cb_group
#         )
#         self.clear_local_costmap_cli = self.create_client(
#             ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap', callback_group=self.cb_group
#         )

#         self.get_logger().info(f"🚀 RTAB-Map DB Switcher Ready. Active floor: [{self.current_floor}]")

#     def _load_config(self, config_path: str):
#         """Loads floor database paths dynamically from floor_config.yaml."""
#         if not os.path.exists(config_path):
#             self.get_logger().error(f"Config file not found at '{config_path}'!")
#             return

#         with open(config_path, 'r') as f:
#             data = yaml.safe_load(f)
#             floors = data.get('floors', {})
#             for floor_name, details in floors.items():
#                 if 'db_path' in details:
#                     self.floor_db_map[floor_name] = os.path.expanduser(details['db_path'])

#         self.get_logger().info(f"Loaded Floor Database Mapping: {self.floor_db_map}")

#     def _map_callback(self, msg: OccupancyGrid):
#         """Tracks the timestamp of incoming /map occupancy grid messages."""
#         self.last_map_stamp = msg.header.stamp

#     def handle_switch_floor_request(self, request, response):
#         """Service callback to handle floor switch requests."""
#         target_floor = "Floor_2" if request.data else "Floor_1"
#         success = self.switch_to_floor_by_name(target_floor)
#         response.success = success
#         response.message = f"Switched active map to {target_floor}" if success else "Switch failed"
#         return response

#     def switch_to_floor_by_name(self, target_floor_name: str) -> bool:
#         """Executes database load, waits for map reload, and flushes costmaps."""
#         if target_floor_name not in self.floor_db_map:
#             self.get_logger().error(f"Target floor '{target_floor_name}' not found in floor_config.yaml!")
#             return False

#         db_path = self.floor_db_map[target_floor_name]
#         if not os.path.exists(db_path):
#             self.get_logger().error(f"Database path '{db_path}' does not exist on disk!")
#             return False

#         self.get_logger().info(f"⏳ Switching database to '{target_floor_name}' ({db_path})...")

#         prev_stamp = self.last_map_stamp

#         # 1. Send load database service request
#         if not self._call_rtabmap_load_db_sync(db_path):
#             self.get_logger().error("Failed during /rtabmap/load_database execution!")
#             return False

#         # 2. Wait up to 10s for RTAB-Map to broadcast a fresh OccupancyGrid on /map
#         self.get_logger().info("Waiting for fresh /map occupancy grid from RTAB-Map...")
#         start_wait = time.time()
#         map_updated = False

#         while time.time() - start_wait < 10.0:
#             if self.last_map_stamp != prev_stamp and self.last_map_stamp is not None:
#                 map_updated = True
#                 self.get_logger().info("Received fresh occupancy grid from RTAB-Map.")
#                 break
#             time.sleep(0.2)

#         if not map_updated:
#             self.get_logger().warn("Timed out waiting for new /map topic update. Proceeding with costmap clear...")

#         # 3. Clear Nav2 costmaps
#         self._clear_nav2_costmaps()
#         time.sleep(0.5)

#         # 4. Update current floor state & publish topic
#         self.current_floor = target_floor_name
#         msg = String()
#         msg.data = self.current_floor
#         self.floor_pub.publish(msg)

#         self.get_logger().info(f"🎉 Floor switch to [{self.current_floor}] successfully finished!")
#         return True

#     def _call_rtabmap_load_db_sync(self, db_path: str) -> bool:
#         """Sends synchronous load_database request to RTAB-Map."""
#         if not self.rtabmap_load_db_cli.wait_for_service(timeout_sec=5.0):
#             self.get_logger().error("Service /rtabmap/load_database is unavailable!")
#             return False

#         req = LoadDatabase.Request()
#         req.database_path = db_path
#         req.clear = False

#         future = self.rtabmap_load_db_cli.call_async(req)
        
#         while not future.done():
#             time.sleep(0.1)

#         res = future.result()
#         return res is not None

#     def _clear_nav2_costmaps(self):
#         """Clears Nav2 global and local costmaps."""
#         req = ClearEntireCostmap.Request()
#         if self.clear_global_costmap_cli.wait_for_service(timeout_sec=2.0):
#             self.clear_global_costmap_cli.call_async(req)
#         if self.clear_local_costmap_cli.wait_for_service(timeout_sec=2.0):
#             self.clear_local_costmap_cli.call_async(req)
#         self.get_logger().info("Nav2 global and local costmaps cleared.")


# def main(args=None):
#     rclpy.init(args=args)
#     node = RtabmapDbSwitcherNode()

#     # MultiThreadedExecutor is required so background callbacks run during blocking loops
#     executor = MultiThreadedExecutor()
#     executor.add_node(node)

#     try:
#         executor.spin()
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()


