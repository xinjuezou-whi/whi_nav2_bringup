#!/usr/bin/env python3
import math
import time
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)

from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
from sensor_msgs.msg import Range
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import GetCostmap, ClearEntireCostmap
from std_srvs.srv import SetBool, Empty


class MultiFloorDispatcherActionServer(Node):

    def __init__(self):
        super().__init__('multi_floor_dispatcher_action_server')

        self._cb_group = ReentrantCallbackGroup()

        # Load parameters and configuration
        self.declare_parameter(
            'config_file',
            '/home/nvidia/ros2_ws/src/whi_nav2_bringup/config/floor_config.yaml'
        )
        self.declare_parameter('default_floor', 'Floor_1')

        config_path = self.get_parameter('config_file').get_parameter_value().string_value
        self.active_floor = self.get_parameter('default_floor').get_parameter_value().string_value

        with open(config_path, 'r') as f:
            self.floor_config = yaml.safe_load(f)['floors']

        self.get_logger().info(f"Loaded config for floors: {list(self.floor_config.keys())}")

        # Nav2 Action Client
        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose', callback_group=self._cb_group
        )

        # Latched QoS Profile
        latch_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        # Publishers
        self.floor_pub = self.create_publisher(String, 'current_floor', latch_qos)
        self.elevator_call_pub = self.create_publisher(String, 'elevator/call_floor', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )

        # Subscribers
        self.floor_sub = self.create_subscription(
            String, 'current_floor', self._current_floor_cb, latch_qos, callback_group=self._cb_group
        )
        self.elevator_status_sub = self.create_subscription(
            String, 'elevator/status', self._elevator_status_cb, 10, callback_group=self._cb_group
        )
        
        # ArUco Marker Pose Subscriber
        self.aruco_pose = None
        self.aruco_sub = self.create_subscription(
            PoseStamped, 'aruco_pose', self._aruco_pose_cb, 10, callback_group=self._cb_group
        )

        # Proximity Sensors (Range)
        self.range_left_front = None
        self.range_right_front = None
        self.left_range_sub = self.create_subscription(
            Range, '/left_front', self._left_range_cb, 10, callback_group=self._cb_group
        )
        self.right_range_sub = self.create_subscription(
            Range, '/right_front', self._right_range_cb, 10, callback_group=self._cb_group
        )

        self.elevator_doors_open = False

        # Service Clients
        self.switch_db_cli = self.create_client(
            SetBool, 'switch_floor_db', callback_group=self._cb_group
        )
        self.get_costmap_cli = self.create_client(
            GetCostmap, '/global_costmap/get_costmap', callback_group=self._cb_group
        )
        self.reset_odom_cli = self.create_client(
            Empty, '/reset_odometry', callback_group=self._cb_group
        )

        # Action Server
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_multi_floor',
            execute_callback=self.execute_action_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._cb_group
        )

        self.get_logger().info(f"🚀 Action Server ready. Initial floor state set to: [{self.active_floor}]")

    def _current_floor_cb(self, msg: String):
        if msg.data in self.floor_config:
            if msg.data != self.active_floor:
                self.get_logger().info(f"🔄 Synced active floor with switcher node: [{msg.data}]")
            self.active_floor = msg.data

    def _elevator_status_cb(self, msg: String):
        if 'DOORS_OPEN' in msg.data:
            self.elevator_doors_open = True

    def _aruco_pose_cb(self, msg: PoseStamped):
        self.aruco_pose = msg

    def _left_range_cb(self, msg: Range):
        self.range_left_front = msg

    def _right_range_cb(self, msg: Range):
        self.range_right_front = msg

    def are_doors_physically_open(self, min_clearance_m: float = 0.8) -> bool:
        """Validates range sensors to confirm front path is physically clear of doors/obstacles."""
        if self.range_left_front is None or self.range_right_front is None:
            self.get_logger().warn("⚠️ Proximity sensor data not received yet. Retrying...")
            return False

        left_dist = self.range_left_front.range
        right_dist = self.range_right_front.range

        # Check if measured distance exceeds clearance threshold or matches max_range (clear pathway)
        left_clear = (left_dist >= min_clearance_m) or (left_dist >= self.range_left_front.max_range)
        right_clear = (right_dist >= min_clearance_m) or (right_dist >= self.range_right_front.max_range)

        if left_clear and right_clear:
            self.get_logger().info(
                f"✅ Proximity check PASSED (Left: {left_dist:.2f}m, Right: {right_dist:.2f}m). Path clear."
            )
            return True
        else:
            self.get_logger().warn(
                f"🛑 Door/Obstacle detected! (Left: {left_dist:.2f}m, Right: {right_dist:.2f}m). Holding..."
            )
            return False

    def goal_callback(self, goal_request):
        self.get_logger().info("Received multi-floor navigation goal request.")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().warn("Goal cancel requested.")
        return CancelResponse.ACCEPT

    def parse_pose_config(self, pose_data):
        if isinstance(pose_data, dict):
            x = float(pose_data.get('x', 0.0))
            y = float(pose_data.get('y', 0.0))
            yaw = float(pose_data.get('yaw', 0.0))

            if pose_data.get('use_degrees', False) or abs(yaw) > math.pi:
                yaw = math.radians(yaw)

            frame_id = str(pose_data.get('frame_id', 'map'))
        elif isinstance(pose_data, (list, tuple)):
            x = float(pose_data[0])
            y = float(pose_data[1])
            yaw = float(pose_data[2]) if len(pose_data) > 2 else 0.0
            if abs(yaw) > math.pi:
                yaw = math.radians(yaw)
            frame_id = 'map'
        else:
            raise ValueError(f"Unsupported pose format in YAML: {type(pose_data)}")
        return x, y, yaw, frame_id

    def publish_initial_pose(self, x: float, y: float, yaw_rad: float = 0.0, frame_id: str = 'map'):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0

        msg.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)

        cov = [0.0] * 36
        cov[0] = 0.25
        cov[7] = 0.25
        cov[14] = 1e-3
        cov[21] = 1e-3
        cov[28] = 1e-3
        cov[35] = 0.06
        msg.pose.covariance = cov

        for _ in range(3):
            self.initial_pose_pub.publish(msg)
            time.sleep(0.05)

        self.get_logger().info(
            f"📍 Published Initial Pose Seed: frame={frame_id}, x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw_rad):.1f}°"
        )

    def clear_all_costmaps(self):
        for srv_name in ['/global_costmap/clear_entirely_global_costmap', '/local_costmap/clear_entirely_local_costmap']:
            cli = self.create_client(ClearEntireCostmap, srv_name, callback_group=self._cb_group)
            if cli.wait_for_service(timeout_sec=1.0):
                req = ClearEntireCostmap.Request()
                cli.call_async(req)
        self.get_logger().info("🧹 Nav2 costmaps cleared.")

    def try_reset_odometry(self):
        if self.reset_odom_cli.wait_for_service(timeout_sec=0.5):
            self.get_logger().info("🔄 Resetting local wheel odometry frame...")
            self.reset_odom_cli.call_async(Empty.Request())

    def wait_for_map_and_costmap_ready(self, timeout_sec: float = 5.0) -> bool:
        if not self.get_costmap_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Global costmap service unavailable, skipping polling.")
            return False

        req = GetCostmap.Request()
        start_t = time.time()

        while (time.time() - start_t) < timeout_sec:
            future = self.get_costmap_cli.call_async(req)
            while not future.done():
                time.sleep(0.05)

            if future.result() is not None:
                return True
            time.sleep(0.1)

        return False

    def send_nav_goal(self, x: float, y: float, yaw_rad: float = 0.0, frame_id: str = 'map') -> bool:
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 action server unavailable!")
            return False

        qz = math.sin(yaw_rad / 2.0)
        qw = math.cos(yaw_rad / 2.0)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = frame_id
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.get_logger().info(f"Dispatching Nav2 Goal -> frame: {frame_id}, x: {x:.2f}, y: {y:.2f}")

        send_future = self._nav_client.send_goal_async(goal_msg)
        while not send_future.done():
            time.sleep(0.1)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by Nav2!")
            return False

        result_future = goal_handle.get_result_async()
        while not result_future.done():
            time.sleep(0.1)

        return result_future.result().status == 4

    def align_with_aruco(self, timeout_sec: float = 10.0) -> bool:
        self.get_logger().info("🎯 Starting ArUco alignment routine...")
        start_time = time.time()

        kp_yaw = 0.8
        kp_y = 0.5

        while (time.time() - start_time) < timeout_sec:
            if self.aruco_pose is None:
                self.get_logger().warn("Waiting for ArUco detection...")
                time.sleep(0.1)
                continue

            now_sec = self.get_clock().now().nanoseconds * 1e-9
            msg_sec = self.aruco_pose.header.stamp.sec + (self.aruco_pose.header.stamp.nanosec * 1e-9)
            if abs(now_sec - msg_sec) > 0.5:
                time.sleep(0.05)
                continue

            y_error = self.aruco_pose.pose.position.y
            qz = self.aruco_pose.pose.orientation.z
            qw = self.aruco_pose.pose.orientation.w
            yaw_error = 2.0 * math.atan2(qz, qw)

            if abs(y_error) < 0.02 and abs(yaw_error) < math.radians(2.0):
                self.get_logger().info("✅ ArUco alignment completed successfully.")
                stop_msg = Twist()
                self.cmd_vel_pub.publish(stop_msg)
                return True

            twist = Twist()
            twist.angular.z = -kp_yaw * yaw_error
            
            if abs(yaw_error) < math.radians(5.0):
                twist.linear.y = -kp_y * y_error

            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)

        self.cmd_vel_pub.publish(Twist())
        self.get_logger().warn("⚠️ ArUco alignment timed out. Proceeding with entry...")
        return False

    def drive_straight_blind(self, distance_m: float = 1.0, speed_mps: float = 0.15, direction_desc: str = "into elevator cabin"):
        self.get_logger().info(f"🤖 Driving blindly forward {distance_m:.2f}m at {speed_mps:.2f}m/s {direction_desc}...")

        duration_sec = distance_m / speed_mps
        rate_hz = 10
        sleep_interval = 1.0 / rate_hz
        steps = int(duration_sec * rate_hz)

        twist = Twist()
        twist.linear.x = float(speed_mps)

        for _ in range(steps):
            self.cmd_vel_pub.publish(twist)
            time.sleep(sleep_interval)

        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)
        time.sleep(0.5)

    def execute_action_callback(self, goal_handle):
        req_pose = goal_handle.request.pose
        target_x = req_pose.pose.position.x
        target_y = req_pose.pose.position.y

        req_frame = req_pose.header.frame_id
        target_floor = req_frame if req_frame in self.floor_config else self.active_floor

        self.get_logger().info(
            f"🎯 Mission: Move from [{self.active_floor}] to [{target_floor}] at x={target_x:.2f}, y={target_y:.2f}"
        )

        # CASE A: Same-Floor Navigation
        if self.active_floor == target_floor:
            self.get_logger().info(f"Already on [{target_floor}]. Navigating directly to target...")
            if self.send_nav_goal(target_x, target_y, frame_id='map'):
                goal_handle.succeed()
                return NavigateToPose.Result()
            else:
                goal_handle.abort()
                return NavigateToPose.Result()

        # CASE B: Cross-Floor Navigation
        cur_floor_cfg = self.floor_config[self.active_floor]
        target_floor_cfg = self.floor_config[target_floor]

        # STAGE 1: Coarse Staging
        coarse_raw = cur_floor_cfg['elevator_entry_pose']
        cx, cy, cyaw_rad, cframe = self.parse_pose_config(coarse_raw)

        self.get_logger().info(
            f"[STAGE 1] Driving to staging pose (x={cx}, y={cy}, yaw={math.degrees(cyaw_rad):.1f}°) on [{self.active_floor}]"
        )
        if not self.send_nav_goal(cx, cy, cyaw_rad, frame_id=cframe):
            self.get_logger().error("Stage 1 failed to reach elevator. Aborting mission.")
            goal_handle.abort()
            return NavigateToPose.Result()

        # STAGE 2: Elevator Handshake, Physical Proximity Check & ArUco Alignment
        self.get_logger().info(f"[STAGE 2] Calling elevator to [{self.active_floor}]")
        self.elevator_call_pub.publish(String(data=target_floor))

        self.elevator_doors_open = False
        start_time = time.time()

        # 1. Wait for string command signal
        while not self.elevator_doors_open:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return NavigateToPose.Result()
            if time.time() - start_time > 360.0:
                self.get_logger().error("Timed out waiting for elevator doors topic signal!")
                goal_handle.abort()
                return NavigateToPose.Result()
            time.sleep(0.5)

        # 2. Safety Validation: Verify physical range sensors show path is clear
        self.get_logger().info("🔍 Performing physical proximity sensor validation before entry...")
        prox_start = time.time()
        while not self.are_doors_physically_open(min_clearance_m=0.8):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return NavigateToPose.Result()
            if time.time() - prox_start > 15.0:
                self.get_logger().error("Proximity sensors show doors/obstacles blocking entry! Aborting.")
                goal_handle.abort()
                return NavigateToPose.Result()
            time.sleep(0.5)

        # 3. Align & Drive in
        # self.align_with_aruco(timeout_sec=10.0)

        entry_cfg = cur_floor_cfg.get('blind_entry', cur_floor_cfg.get('blind_exit', {}))
        entry_dist = float(entry_cfg.get('distance', 1.0))
        entry_speed = float(entry_cfg.get('speed', 0.15))
        self.drive_straight_blind(distance_m=entry_dist, speed_mps=entry_speed, direction_desc="into cabin")

        self.elevator_call_pub.publish(String(data=target_floor))

        t_switch_start = time.time()

        if not self.switch_db_cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("switch_floor_db service unavailable!")
            goal_handle.abort()
            return NavigateToPose.Result()

        self.get_logger().info("⚡ Triggering background database switch during elevator transit...")
        req = SetBool.Request()
        req.data = (target_floor == "Floor_2")
        db_switch_future = self.switch_db_cli.call_async(req)

        self.elevator_doors_open = False
        start_time = time.time()

        while not self.elevator_doors_open:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return NavigateToPose.Result()
            if time.time() - start_time > 600.0:
                self.get_logger().error(f"Timed out waiting to arrive at [{target_floor}]!")
                goal_handle.abort()
                return NavigateToPose.Result()
            time.sleep(0.5)

        # STAGE 3: Exit Cabin, Flush TF/Costmaps & Seed ICP Pose
        # Optional: Physical proximity check for exiting cabin as well
        self.get_logger().info("🔍 Performing physical proximity sensor validation before exit...")
        prox_exit_start = time.time()
        while not self.are_doors_physically_open(min_clearance_m=0.8):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return NavigateToPose.Result()
            if time.time() - prox_exit_start > 15.0:
                self.get_logger().error("Proximity sensors show doors/obstacles blocking exit! Aborting.")
                goal_handle.abort()
                return NavigateToPose.Result()
            time.sleep(0.5)

        exit_cfg = target_floor_cfg.get('blind_exit', {})
        exit_dist = float(exit_cfg.get('distance', 1.0))
        exit_speed = float(exit_cfg.get('speed', 0.15))
        self.drive_straight_blind(distance_m=exit_dist, speed_mps=exit_speed, direction_desc="out of cabin")

        while rclpy.ok() and not db_switch_future.done():
            time.sleep(0.05)

        db_switch_duration = time.time() - t_switch_start
        self.get_logger().info(f"⏱️ [METRIC] Map DB Switch completed in: {db_switch_duration:.3f} seconds")

        time.sleep(1.0)
        self.try_reset_odometry()
        self.clear_all_costmaps()

        landing_raw = target_floor_cfg.get('landing_pose', target_floor_cfg.get('elevator_entry_pose'))
        lx, ly, lyaw_rad, lframe = self.parse_pose_config(landing_raw)
        
        t_icp_start = time.time()
        self.publish_initial_pose(lx, ly, lyaw_rad, frame_id=lframe)

        self.get_logger().info("⏳ Holding robot stationary for ICP point cloud registration lock...")
        time.sleep(3.0) 

        costmap_ready = self.wait_for_map_and_costmap_ready(timeout_sec=3.0)
        
        t_map_ready_duration = time.time() - t_icp_start
        total_transition_latency = time.time() - t_switch_start

        if costmap_ready:
            self.get_logger().info(
                f"⏱️ [METRIC] Map & Costmap ready for planning in: {t_map_ready_duration:.3f} seconds (post-initialpose)"
            )
        else:
            self.get_logger().warn("⚠️ Costmap service timed out. Proceeding with goal attempt...")

        self.get_logger().info(
            f"⏱️ [METRIC] Total Switch-to-Ready Transition Latency: {total_transition_latency:.3f} seconds"
        )

        # STAGE 4: Execute Final Goal
        self.get_logger().info(f"[STAGE 4] Navigating to final target on [{target_floor}]")
        if self.send_nav_goal(target_x, target_y, yaw_rad=0.0, frame_id='map'):
            goal_handle.succeed()
            self.get_logger().info("🎉 Multi-floor mission completed successfully!")
        else:
            self.get_logger().error("Stage 4 final approach failed.")
            goal_handle.abort()

        return NavigateToPose.Result()


def main():
    rclpy.init()
    node = MultiFloorDispatcherActionServer()

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

























###########################################################################################
# import math
# import time
# import yaml
# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
# from rclpy.executors import MultiThreadedExecutor
# from rclpy.callback_groups import ReentrantCallbackGroup
# from rclpy.qos import (
#     QoSProfile,
#     QoSDurabilityPolicy,
#     QoSReliabilityPolicy,
#     QoSHistoryPolicy,
# )

# from std_msgs.msg import String
# from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
# from nav2_msgs.action import NavigateToPose
# from nav2_msgs.srv import GetCostmap, ClearEntireCostmap
# from std_srvs.srv import SetBool, Empty


# class MultiFloorDispatcherActionServer(Node):

#     def __init__(self):
#         super().__init__('multi_floor_dispatcher_action_server')

#         self._cb_group = ReentrantCallbackGroup()

#         # Load parameters and configuration
#         self.declare_parameter(
#             'config_file',
#             '/home/nvidia/ros2_ws/src/whi_nav2_bringup/config/floor_config.yaml'
#         )
#         self.declare_parameter('default_floor', 'Floor_1')

#         config_path = self.get_parameter('config_file').get_parameter_value().string_value
#         self.active_floor = self.get_parameter('default_floor').get_parameter_value().string_value

#         with open(config_path, 'r') as f:
#             self.floor_config = yaml.safe_load(f)['floors']

#         self.get_logger().info(f"Loaded config for floors: {list(self.floor_config.keys())}")

#         # Nav2 Action Client
#         self._nav_client = ActionClient(
#             self, NavigateToPose, 'navigate_to_pose', callback_group=self._cb_group
#         )

#         # Latched QoS Profile
#         latch_qos = QoSProfile(
#             depth=1,
#             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
#             reliability=QoSReliabilityPolicy.RELIABLE,
#             history=QoSHistoryPolicy.KEEP_LAST,
#         )

#         # Publishers
#         self.floor_pub = self.create_publisher(String, 'current_floor', latch_qos)
#         self.elevator_call_pub = self.create_publisher(String, 'elevator/call_floor', 10)
#         self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
#         self.initial_pose_pub = self.create_publisher(
#             PoseWithCovarianceStamped, '/initialpose', 10
#         )

#         # Subscribers - Automatically receives latched /current_floor state from switcher
#         self.floor_sub = self.create_subscription(
#             String, 'current_floor', self._current_floor_cb, latch_qos, callback_group=self._cb_group
#         )
#         self.elevator_status_sub = self.create_subscription(
#             String, 'elevator/status', self._elevator_status_cb, 10, callback_group=self._cb_group
#         )
        
#         # ArUco Marker Pose Subscriber
#         self.aruco_pose = None
#         self.aruco_sub = self.create_subscription(
#             PoseStamped, 'aruco_pose', self._aruco_pose_cb, 10, callback_group=self._cb_group
#         )

#         self.elevator_doors_open = False

#         # Service Clients
#         self.switch_db_cli = self.create_client(
#             SetBool, 'switch_floor_db', callback_group=self._cb_group
#         )
#         self.get_costmap_cli = self.create_client(
#             GetCostmap, '/global_costmap/get_costmap', callback_group=self._cb_group
#         )
#         self.reset_odom_cli = self.create_client(
#             Empty, '/reset_odometry', callback_group=self._cb_group
#         )

#         # Action Server
#         self._action_server = ActionServer(
#             self,
#             NavigateToPose,
#             'navigate_multi_floor',
#             execute_callback=self.execute_action_callback,
#             goal_callback=self.goal_callback,
#             cancel_callback=self.cancel_callback,
#             callback_group=self._cb_group
#         )

#         self.get_logger().info(f"🚀 Action Server ready. Initial floor state set to: [{self.active_floor}]")

#     def _current_floor_cb(self, msg: String):
#         """Syncs active floor immediately upon receiving latched floor topic."""
#         if msg.data in self.floor_config:
#             if msg.data != self.active_floor:
#                 self.get_logger().info(f"🔄 Synced active floor with switcher node: [{msg.data}]")
#             self.active_floor = msg.data

#     def _elevator_status_cb(self, msg: String):
#         if 'DOORS_OPEN' in msg.data:
#             self.elevator_doors_open = True

#     def _aruco_pose_cb(self, msg: PoseStamped):
#         self.aruco_pose = msg

#     def goal_callback(self, goal_request):
#         self.get_logger().info("Received multi-floor navigation goal request.")
#         return GoalResponse.ACCEPT

#     def cancel_callback(self, goal_handle):
#         self.get_logger().warn("Goal cancel requested.")
#         return CancelResponse.ACCEPT

#     def parse_pose_config(self, pose_data):
#         if isinstance(pose_data, dict):
#             x = float(pose_data.get('x', 0.0))
#             y = float(pose_data.get('y', 0.0))
#             yaw = float(pose_data.get('yaw', 0.0))

#             if pose_data.get('use_degrees', False) or abs(yaw) > math.pi:
#                 yaw = math.radians(yaw)

#             frame_id = str(pose_data.get('frame_id', 'map'))
#         elif isinstance(pose_data, (list, tuple)):
#             x = float(pose_data[0])
#             y = float(pose_data[1])
#             yaw = float(pose_data[2]) if len(pose_data) > 2 else 0.0
#             if abs(yaw) > math.pi:
#                 yaw = math.radians(yaw)
#             frame_id = 'map'
#         else:
#             raise ValueError(f"Unsupported pose format in YAML: {type(pose_data)}")
#         return x, y, yaw, frame_id

#     def publish_initial_pose(self, x: float, y: float, yaw_rad: float = 0.0, frame_id: str = 'map'):
#         msg = PoseWithCovarianceStamped()
#         msg.header.frame_id = frame_id
#         msg.header.stamp = self.get_clock().now().to_msg()
#         msg.pose.pose.position.x = float(x)
#         msg.pose.pose.position.y = float(y)
#         msg.pose.pose.position.z = 0.0

#         msg.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
#         msg.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)

#         cov = [0.0] * 36
#         cov[0] = 0.25
#         cov[7] = 0.25
#         cov[14] = 1e-3
#         cov[21] = 1e-3
#         cov[28] = 1e-3
#         cov[35] = 0.06
#         msg.pose.covariance = cov

#         for _ in range(3):
#             self.initial_pose_pub.publish(msg)
#             time.sleep(0.05)

#         self.get_logger().info(
#             f"📍 Published Initial Pose Seed: frame={frame_id}, x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw_rad):.1f}°"
#         )

#     def clear_all_costmaps(self):
#         for srv_name in ['/global_costmap/clear_entirely_global_costmap', '/local_costmap/clear_entirely_local_costmap']:
#             cli = self.create_client(ClearEntireCostmap, srv_name, callback_group=self._cb_group)
#             if cli.wait_for_service(timeout_sec=1.0):
#                 req = ClearEntireCostmap.Request()
#                 cli.call_async(req)
#         self.get_logger().info("🧹 Nav2 costmaps cleared.")

#     def try_reset_odometry(self):
#         if self.reset_odom_cli.wait_for_service(timeout_sec=0.5):
#             self.get_logger().info("🔄 Resetting local wheel odometry frame...")
#             self.reset_odom_cli.call_async(Empty.Request())

#     def wait_for_map_and_costmap_ready(self, timeout_sec: float = 5.0) -> bool:
#         if not self.get_costmap_cli.wait_for_service(timeout_sec=2.0):
#             self.get_logger().warn("Global costmap service unavailable, skipping polling.")
#             return False

#         req = GetCostmap.Request()
#         start_t = time.time()

#         while (time.time() - start_t) < timeout_sec:
#             future = self.get_costmap_cli.call_async(req)
#             while not future.done():
#                 time.sleep(0.05)

#             if future.result() is not None:
#                 return True
#             time.sleep(0.1)

#         return False

#     def send_nav_goal(self, x: float, y: float, yaw_rad: float = 0.0, frame_id: str = 'map') -> bool:
#         if not self._nav_client.wait_for_server(timeout_sec=5.0):
#             self.get_logger().error("Nav2 action server unavailable!")
#             return False

#         qz = math.sin(yaw_rad / 2.0)
#         qw = math.cos(yaw_rad / 2.0)

#         goal_msg = NavigateToPose.Goal()
#         goal_msg.pose.header.frame_id = frame_id
#         goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
#         goal_msg.pose.pose.position.x = float(x)
#         goal_msg.pose.pose.position.y = float(y)
#         goal_msg.pose.pose.orientation.z = qz
#         goal_msg.pose.pose.orientation.w = qw

#         self.get_logger().info(f"Dispatching Nav2 Goal -> frame: {frame_id}, x: {x:.2f}, y: {y:.2f}")

#         send_future = self._nav_client.send_goal_async(goal_msg)
#         while not send_future.done():
#             time.sleep(0.1)

#         goal_handle = send_future.result()
#         if not goal_handle.accepted:
#             self.get_logger().error("Goal rejected by Nav2!")
#             return False

#         result_future = goal_handle.get_result_async()
#         while not result_future.done():
#             time.sleep(0.1)

#         return result_future.result().status == 4

#     def align_with_aruco(self, timeout_sec: float = 10.0) -> bool:
#         self.get_logger().info("🎯 Starting ArUco alignment routine...")
#         start_time = time.time()

#         kp_yaw = 0.8
#         kp_y = 0.5

#         while (time.time() - start_time) < timeout_sec:
#             if self.aruco_pose is None:
#                 self.get_logger().warn("Waiting for ArUco detection...")
#                 time.sleep(0.1)
#                 continue

#             now_sec = self.get_clock().now().nanoseconds * 1e-9
#             msg_sec = self.aruco_pose.header.stamp.sec + (self.aruco_pose.header.stamp.nanosec * 1e-9)
#             if abs(now_sec - msg_sec) > 0.5:
#                 time.sleep(0.05)
#                 continue

#             y_error = self.aruco_pose.pose.position.y
#             qz = self.aruco_pose.pose.orientation.z
#             qw = self.aruco_pose.pose.orientation.w
#             yaw_error = 2.0 * math.atan2(qz, qw)

#             if abs(y_error) < 0.02 and abs(yaw_error) < math.radians(2.0):
#                 self.get_logger().info("✅ ArUco alignment completed successfully.")
#                 stop_msg = Twist()
#                 self.cmd_vel_pub.publish(stop_msg)
#                 return True

#             twist = Twist()
#             twist.angular.z = -kp_yaw * yaw_error
            
#             if abs(yaw_error) < math.radians(5.0):
#                 twist.linear.y = -kp_y * y_error

#             self.cmd_vel_pub.publish(twist)
#             time.sleep(0.05)

#         self.cmd_vel_pub.publish(Twist())
#         self.get_logger().warn("⚠️ ArUco alignment timed out. Proceeding with entry...")
#         return False

#     def drive_straight_blind(self, distance_m: float = 1.0, speed_mps: float = 0.15, direction_desc: str = "into elevator cabin"):
#         self.get_logger().info(f"🤖 Driving blindly forward {distance_m:.2f}m at {speed_mps:.2f}m/s {direction_desc}...")

#         duration_sec = distance_m / speed_mps
#         rate_hz = 10
#         sleep_interval = 1.0 / rate_hz
#         steps = int(duration_sec * rate_hz)

#         twist = Twist()
#         twist.linear.x = float(speed_mps)

#         for _ in range(steps):
#             self.cmd_vel_pub.publish(twist)
#             time.sleep(sleep_interval)

#         stop_msg = Twist()
#         self.cmd_vel_pub.publish(stop_msg)
#         time.sleep(0.5)

#     def execute_action_callback(self, goal_handle):
#         req_pose = goal_handle.request.pose
#         target_x = req_pose.pose.position.x
#         target_y = req_pose.pose.position.y

#         req_frame = req_pose.header.frame_id
#         target_floor = req_frame if req_frame in self.floor_config else self.active_floor

#         self.get_logger().info(
#             f"🎯 Mission: Move from [{self.active_floor}] to [{target_floor}] at x={target_x:.2f}, y={target_y:.2f}"
#         )

#         # CASE A: Same-Floor Navigation
#         if self.active_floor == target_floor:
#             self.get_logger().info(f"Already on [{target_floor}]. Navigating directly to target...")
#             if self.send_nav_goal(target_x, target_y, frame_id='map'):
#                 goal_handle.succeed()
#                 return NavigateToPose.Result()
#             else:
#                 goal_handle.abort()
#                 return NavigateToPose.Result()

#         # CASE B: Cross-Floor Navigation
#         cur_floor_cfg = self.floor_config[self.active_floor]
#         target_floor_cfg = self.floor_config[target_floor]

#         # STAGE 1: Coarse Staging
#         coarse_raw = cur_floor_cfg['elevator_entry_pose']
#         cx, cy, cyaw_rad, cframe = self.parse_pose_config(coarse_raw)

#         self.get_logger().info(
#             f"[STAGE 1] Driving to staging pose (x={cx}, y={cy}, yaw={math.degrees(cyaw_rad):.1f}°) on [{self.active_floor}]"
#         )
#         if not self.send_nav_goal(cx, cy, cyaw_rad, frame_id=cframe):
#             self.get_logger().error("Stage 1 failed to reach elevator. Aborting mission.")
#             goal_handle.abort()
#             return NavigateToPose.Result()

#         # STAGE 2: Elevator Handshake, ArUco Alignment & Parallel DB Switch
#         self.get_logger().info(f"[STAGE 2] Calling elevator to [{self.active_floor}]")
#         self.elevator_call_pub.publish(String(data=target_floor))

#         self.elevator_doors_open = False
#         start_time = time.time()

#         while not self.elevator_doors_open:
#             if goal_handle.is_cancel_requested:
#                 goal_handle.canceled()
#                 return NavigateToPose.Result()
#             if time.time() - start_time > 600.0:
#                 self.get_logger().error("Timed out waiting for elevator doors!")
#                 goal_handle.abort()
#                 return NavigateToPose.Result()
#             time.sleep(0.5)

#         #self.align_with_aruco(timeout_sec=10.0)

#         entry_cfg = cur_floor_cfg.get('blind_entry', cur_floor_cfg.get('blind_exit', {}))
#         entry_dist = float(entry_cfg.get('distance', 1.0))
#         entry_speed = float(entry_cfg.get('speed', 0.15))
#         self.drive_straight_blind(distance_m=entry_dist, speed_mps=entry_speed, direction_desc="into cabin")

#         self.elevator_call_pub.publish(String(data=target_floor))

#         t_switch_start = time.time()

#         if not self.switch_db_cli.wait_for_service(timeout_sec=3.0):
#             self.get_logger().error("switch_floor_db service unavailable!")
#             goal_handle.abort()
#             return NavigateToPose.Result()

#         self.get_logger().info("⚡ Triggering background database switch during elevator transit...")
#         req = SetBool.Request()
#         req.data = (target_floor == "Floor_2")
#         db_switch_future = self.switch_db_cli.call_async(req)

#         self.elevator_doors_open = False
#         start_time = time.time()

#         while not self.elevator_doors_open:
#             if goal_handle.is_cancel_requested:
#                 goal_handle.canceled()
#                 return NavigateToPose.Result()
#             if time.time() - start_time > 600.0:
#                 self.get_logger().error(f"Timed out waiting to arrive at [{target_floor}]!")
#                 goal_handle.abort()
#                 return NavigateToPose.Result()
#             time.sleep(0.5)

#         # STAGE 3: Exit Cabin, Flush TF/Costmaps & Seed ICP Pose
#         exit_cfg = target_floor_cfg.get('blind_exit', {})
#         exit_dist = float(exit_cfg.get('distance', 1.0))
#         exit_speed = float(exit_cfg.get('speed', 0.15))
#         self.drive_straight_blind(distance_m=exit_dist, speed_mps=exit_speed, direction_desc="out of cabin")

#         while rclpy.ok() and not db_switch_future.done():
#             time.sleep(0.05)

#         db_switch_duration = time.time() - t_switch_start
#         self.get_logger().info(f"⏱️ [METRIC] Map DB Switch completed in: {db_switch_duration:.3f} seconds")

#         time.sleep(1.0)
#         self.try_reset_odometry()
#         self.clear_all_costmaps()

#         landing_raw = target_floor_cfg.get('landing_pose', target_floor_cfg.get('elevator_entry_pose'))
#         lx, ly, lyaw_rad, lframe = self.parse_pose_config(landing_raw)
        
#         t_icp_start = time.time()
#         self.publish_initial_pose(lx, ly, lyaw_rad, frame_id=lframe)

#         self.get_logger().info("⏳ Holding robot stationary for ICP point cloud registration lock...")
#         time.sleep(3.0) 

#         costmap_ready = self.wait_for_map_and_costmap_ready(timeout_sec=3.0)
        
#         t_map_ready_duration = time.time() - t_icp_start
#         total_transition_latency = time.time() - t_switch_start

#         if costmap_ready:
#             self.get_logger().info(
#                 f"⏱️ [METRIC] Map & Costmap ready for planning in: {t_map_ready_duration:.3f} seconds (post-initialpose)"
#             )
#         else:
#             self.get_logger().warn("⚠️ Costmap service timed out. Proceeding with goal attempt...")

#         self.get_logger().info(
#             f"⏱️ [METRIC] Total Switch-to-Ready Transition Latency: {total_transition_latency:.3f} seconds"
#         )

#         # STAGE 4: Execute Final Goal
#         self.get_logger().info(f"[STAGE 4] Navigating to final target on [{target_floor}]")
#         if self.send_nav_goal(target_x, target_y, yaw_rad=0.0, frame_id='map'):
#             goal_handle.succeed()
#             self.get_logger().info("🎉 Multi-floor mission completed successfully!")
#         else:
#             self.get_logger().error("Stage 4 final approach failed.")
#             goal_handle.abort()

#         return NavigateToPose.Result()


# def main():
#     rclpy.init()
#     node = MultiFloorDispatcherActionServer()

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



















#####################################################################################################
# import math
# import time
# import yaml
# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
# from rclpy.executors import MultiThreadedExecutor
# from rclpy.callback_groups import ReentrantCallbackGroup
# from rclpy.qos import (
#     QoSProfile,
#     QoSDurabilityPolicy,
#     QoSReliabilityPolicy,
#     QoSHistoryPolicy,
# )

# from std_msgs.msg import String
# from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
# from nav2_msgs.action import NavigateToPose
# from nav2_msgs.srv import GetCostmap, ClearEntireCostmap
# from std_srvs.srv import SetBool, Empty


# class MultiFloorDispatcherActionServer(Node):

#     def __init__(self):
#         super().__init__('multi_floor_dispatcher_action_server')

#         self._cb_group = ReentrantCallbackGroup()

#         # Load parameters and configuration
#         self.declare_parameter(
#             'config_file',
#             '/home/nvidia/ros2_ws/src/whi_nav2_bringup/config/floor_config.yaml'
#         )
#         self.declare_parameter('default_floor', 'Floor_1')

#         config_path = self.get_parameter('config_file').get_parameter_value().string_value
#         self.active_floor = self.get_parameter('default_floor').get_parameter_value().string_value

#         with open(config_path, 'r') as f:
#             self.floor_config = yaml.safe_load(f)['floors']

#         self.get_logger().info(f"Loaded config for floors: {list(self.floor_config.keys())}")

#         # Nav2 Action Client
#         self._nav_client = ActionClient(
#             self, NavigateToPose, 'navigate_to_pose', callback_group=self._cb_group
#         )

#         # Latched QoS Profile
#         latch_qos = QoSProfile(
#             depth=1,
#             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
#             reliability=QoSReliabilityPolicy.RELIABLE,
#             history=QoSHistoryPolicy.KEEP_LAST,
#         )

#         # Publishers
#         self.floor_pub = self.create_publisher(String, 'current_floor', latch_qos)
#         self.elevator_call_pub = self.create_publisher(String, 'elevator/call_floor', 10)
#         self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
#         self.initial_pose_pub = self.create_publisher(
#             PoseWithCovarianceStamped, '/initialpose', 10
#         )

#         # Subscribers
#         self.floor_sub = self.create_subscription(
#             String, 'current_floor', self._current_floor_cb, 10, callback_group=self._cb_group
#         )
#         self.elevator_status_sub = self.create_subscription(
#             String, 'elevator/status', self._elevator_status_cb, 10, callback_group=self._cb_group
#         )

#         self.elevator_doors_open = False

#         # Service Clients
#         self.switch_db_cli = self.create_client(
#             SetBool, 'switch_floor_db', callback_group=self._cb_group
#         )
#         self.get_costmap_cli = self.create_client(
#             GetCostmap, '/global_costmap/get_costmap', callback_group=self._cb_group
#         )
#         self.reset_odom_cli = self.create_client(
#             Empty, '/reset_odometry', callback_group=self._cb_group
#         )

#         # Action Server
#         self._action_server = ActionServer(
#             self,
#             NavigateToPose,
#             'navigate_multi_floor',
#             execute_callback=self.execute_action_callback,
#             goal_callback=self.goal_callback,
#             cancel_callback=self.cancel_callback,
#             callback_group=self._cb_group
#         )

#         self.get_logger().info(f"🚀 Action Server ready. Active floor: [{self.active_floor}]")

#     def _current_floor_cb(self, msg: String):
#         if msg.data in self.floor_config and msg.data != self.active_floor:
#             self.get_logger().info(f"Updated active floor state to: [{msg.data}]")
#             self.active_floor = msg.data

#     def _elevator_status_cb(self, msg: String):
#         if 'DOORS_OPEN' in msg.data:
#             self.elevator_doors_open = True

#     def goal_callback(self, goal_request):
#         self.get_logger().info("Received multi-floor navigation goal request.")
#         return GoalResponse.ACCEPT

#     def cancel_callback(self, goal_handle):
#         self.get_logger().warn("Goal cancel requested.")
#         return CancelResponse.ACCEPT

#     def parse_pose_config(self, pose_data):
#         if isinstance(pose_data, dict):
#             x = float(pose_data.get('x', 0.0))
#             y = float(pose_data.get('y', 0.0))
#             yaw = float(pose_data.get('yaw', 0.0))

#             # Auto-detect degrees if use_degrees is true or if magnitude > pi
#             if pose_data.get('use_degrees', False) or abs(yaw) > math.pi:
#                 yaw = math.radians(yaw)

#             frame_id = str(pose_data.get('frame_id', 'map'))
#         elif isinstance(pose_data, (list, tuple)):
#             x = float(pose_data[0])
#             y = float(pose_data[1])
#             yaw = float(pose_data[2]) if len(pose_data) > 2 else 0.0
#             if abs(yaw) > math.pi:
#                 yaw = math.radians(yaw)
#             frame_id = 'map'
#         else:
#             raise ValueError(f"Unsupported pose format in YAML: {type(pose_data)}")
#         return x, y, yaw, frame_id

#     def publish_initial_pose(self, x: float, y: float, yaw_rad: float = 0.0, frame_id: str = 'map'):
#         """Publishes initial pose seed to trigger ICP point cloud alignment."""
#         msg = PoseWithCovarianceStamped()
#         msg.header.frame_id = frame_id
#         msg.header.stamp = self.get_clock().now().to_msg()
#         msg.pose.pose.position.x = float(x)
#         msg.pose.pose.position.y = float(y)
#         msg.pose.pose.position.z = 0.0

#         msg.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
#         msg.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)

#         # Full 6x6 Covariance Matrix (36 elements)
#         cov = [0.0] * 36
#         cov[0] = 0.25   # X variance (0.5m std dev)
#         cov[7] = 0.25   # Y variance (0.5m std dev)
#         cov[14] = 1e-3  # Z variance
#         cov[21] = 1e-3  # Roll variance
#         cov[28] = 1e-3  # Pitch variance
#         cov[35] = 0.06  # Yaw variance (~14 deg std dev)
#         msg.pose.covariance = cov

#         # Burst publish 3 times over ROS DDS
#         for _ in range(3):
#             self.initial_pose_pub.publish(msg)
#             time.sleep(0.05)

#         self.get_logger().info(
#             f"📍 Published Initial Pose Seed: frame={frame_id}, x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw_rad):.1f}° ({yaw_rad:.3f} rad)"
#         )

#     def clear_all_costmaps(self):
#         """Purges old floor obstacle geometry from Nav2 memory."""
#         for srv_name in ['/global_costmap/clear_entirely_global_costmap', '/local_costmap/clear_entirely_local_costmap']:
#             cli = self.create_client(ClearEntireCostmap, srv_name, callback_group=self._cb_group)
#             if cli.wait_for_service(timeout_sec=1.0):
#                 req = ClearEntireCostmap.Request()
#                 cli.call_async(req)
#         self.get_logger().info("🧹 Nav2 costmaps cleared.")

#     def try_reset_odometry(self):
#         """Attempts to reset local wheel odometry if service is available."""
#         if self.reset_odom_cli.wait_for_service(timeout_sec=0.5):
#             self.get_logger().info("🔄 Resetting local wheel odometry frame...")
#             self.reset_odom_cli.call_async(Empty.Request())

#     def wait_for_map_and_costmap_ready(self, timeout_sec: float = 5.0) -> bool:
#         """Polls global costmap service to verify map readiness for path planning."""
#         if not self.get_costmap_cli.wait_for_service(timeout_sec=2.0):
#             self.get_logger().warn("Global costmap service unavailable, skipping polling.")
#             return False

#         req = GetCostmap.Request()
#         start_t = time.time()

#         while (time.time() - start_t) < timeout_sec:
#             future = self.get_costmap_cli.call_async(req)
#             while not future.done():
#                 time.sleep(0.05)

#             if future.result() is not None:
#                 return True
#             time.sleep(0.1)

#         return False

#     def send_nav_goal(self, x: float, y: float, yaw_rad: float = 0.0, frame_id: str = 'map') -> bool:
#         if not self._nav_client.wait_for_server(timeout_sec=5.0):
#             self.get_logger().error("Nav2 action server unavailable!")
#             return False

#         qz = math.sin(yaw_rad / 2.0)
#         qw = math.cos(yaw_rad / 2.0)

#         goal_msg = NavigateToPose.Goal()
#         goal_msg.pose.header.frame_id = frame_id
#         goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
#         goal_msg.pose.pose.position.x = float(x)
#         goal_msg.pose.pose.position.y = float(y)
#         goal_msg.pose.pose.orientation.z = qz
#         goal_msg.pose.pose.orientation.w = qw

#         self.get_logger().info(f"Dispatching Nav2 Goal -> frame: {frame_id}, x: {x:.2f}, y: {y:.2f}")

#         send_future = self._nav_client.send_goal_async(goal_msg)
#         while not send_future.done():
#             time.sleep(0.1)

#         goal_handle = send_future.result()
#         if not goal_handle.accepted:
#             self.get_logger().error("Goal rejected by Nav2!")
#             return False

#         result_future = goal_handle.get_result_async()
#         while not result_future.done():
#             time.sleep(0.1)

#         return result_future.result().status == 4  # SUCCEEDED

#     def drive_straight_blind(self, distance_m: float = 1.0, speed_mps: float = 0.15, direction_desc: str = "into elevator cabin"):
#         """Publishes velocity commands to drive blindly on wheel odometry."""
#         self.get_logger().info(f"🤖 Driving blindly forward {distance_m:.2f}m at {speed_mps:.2f}m/s {direction_desc}...")

#         duration_sec = distance_m / speed_mps
#         rate_hz = 10
#         sleep_interval = 1.0 / rate_hz
#         steps = int(duration_sec * rate_hz)

#         twist = Twist()
#         twist.linear.x = float(speed_mps)

#         for _ in range(steps):
#             self.cmd_vel_pub.publish(twist)
#             time.sleep(sleep_interval)

#         # Bring robot to complete physical stop
#         stop_msg = Twist()
#         self.cmd_vel_pub.publish(stop_msg)
#         time.sleep(0.5)

#     def execute_action_callback(self, goal_handle):
#         req_pose = goal_handle.request.pose
#         target_x = req_pose.pose.position.x
#         target_y = req_pose.pose.position.y

#         req_frame = req_pose.header.frame_id
#         target_floor = req_frame if req_frame in self.floor_config else self.active_floor

#         self.get_logger().info(
#             f"🎯 Mission: Move from [{self.active_floor}] to [{target_floor}] at x={target_x:.2f}, y={target_y:.2f}"
#         )

#         # -------------------------------------------------------------
#         # CASE A: Same-Floor Navigation
#         # -------------------------------------------------------------
#         if self.active_floor == target_floor:
#             self.get_logger().info(f"Already on [{target_floor}]. Navigating directly to target...")
#             if self.send_nav_goal(target_x, target_y, frame_id='map'):
#                 goal_handle.succeed()
#                 return NavigateToPose.Result()
#             else:
#                 goal_handle.abort()
#                 return NavigateToPose.Result()

#         # -------------------------------------------------------------
#         # CASE B: Cross-Floor Navigation
#         # -------------------------------------------------------------
#         cur_floor_cfg = self.floor_config[self.active_floor]
#         target_floor_cfg = self.floor_config[target_floor]

#         # STAGE 1: Coarse Staging
#         coarse_raw = cur_floor_cfg['elevator_entry_pose']
#         cx, cy, cyaw_rad, cframe = self.parse_pose_config(coarse_raw)

#         self.get_logger().info(
#             f"[STAGE 1] Driving to staging pose (x={cx}, y={cy}, yaw={math.degrees(cyaw_rad):.1f}°) on [{self.active_floor}]"
#         )
#         if not self.send_nav_goal(cx, cy, cyaw_rad, frame_id=cframe):
#             self.get_logger().error("Stage 1 failed to reach elevator. Aborting mission.")
#             goal_handle.abort()
#             return NavigateToPose.Result()

#         # STAGE 2: Elevator Handshake & Parallel DB Switch
#         self.get_logger().info(f"[STAGE 2] Calling elevator to [{self.active_floor}]")
#         self.elevator_call_pub.publish(String(data=target_floor))

#         self.elevator_doors_open = False
#         start_time = time.time()

#         while not self.elevator_doors_open:
#             if goal_handle.is_cancel_requested:
#                 goal_handle.canceled()
#                 return NavigateToPose.Result()
#             if time.time() - start_time > 600.0:
#                 self.get_logger().error("Timed out waiting for elevator doors!")
#                 goal_handle.abort()
#                 return NavigateToPose.Result()
#             time.sleep(0.5)

#         # Doors Open -> Drive blindly in
#         entry_cfg = cur_floor_cfg.get('blind_entry', cur_floor_cfg.get('blind_exit', {}))
#         entry_dist = float(entry_cfg.get('distance', 1.0))
#         entry_speed = float(entry_cfg.get('speed', 0.15))
#         self.drive_straight_blind(distance_m=entry_dist, speed_mps=entry_speed, direction_desc="into cabin")

#         self.elevator_call_pub.publish(String(data=target_floor))

#         # Start timer for DB switch overhead
#         t_switch_start = time.time()

#         if not self.switch_db_cli.wait_for_service(timeout_sec=3.0):
#             self.get_logger().error("switch_floor_db service unavailable!")
#             goal_handle.abort()
#             return NavigateToPose.Result()

#         self.get_logger().info("⚡ Triggering background database switch during elevator transit...")
#         req = SetBool.Request()
#         req.data = (target_floor == "Floor_2")
#         db_switch_future = self.switch_db_cli.call_async(req)

#         # Wait for transit arrival
#         self.elevator_doors_open = False
#         start_time = time.time()

#         while not self.elevator_doors_open:
#             if goal_handle.is_cancel_requested:
#                 goal_handle.canceled()
#                 return NavigateToPose.Result()
#             if time.time() - start_time > 600.0:
#                 self.get_logger().error(f"Timed out waiting to arrive at [{target_floor}]!")
#                 goal_handle.abort()
#                 return NavigateToPose.Result()
#             time.sleep(0.5)

#         # STAGE 3: Exit Cabin, Flush TF/Costmaps & Seed ICP Pose
#         exit_cfg = target_floor_cfg.get('blind_exit', {})
#         exit_dist = float(exit_cfg.get('distance', 1.0))
#         exit_speed = float(exit_cfg.get('speed', 0.15))
#         self.drive_straight_blind(distance_m=exit_dist, speed_mps=exit_speed, direction_desc="out of cabin")

#         # Synchronize DB switch and record switch duration
#         while rclpy.ok() and not db_switch_future.done():
#             time.sleep(0.05)

#         db_switch_duration = time.time() - t_switch_start
#         self.get_logger().info(f"⏱️ [METRIC] Map DB Switch completed in: {db_switch_duration:.3f} seconds")

#         # Step A: Allow wheel odometry and physical robot motion to come to a complete stop
#         time.sleep(1.0)
#         self.try_reset_odometry()

#         # Step B: Flush costmap obstacles left over from previous floor
#         self.clear_all_costmaps()

#         # Step C: Seed Initial Pose
#         landing_raw = target_floor_cfg.get('landing_pose', target_floor_cfg.get('elevator_entry_pose'))
#         lx, ly, lyaw_rad, lframe = self.parse_pose_config(landing_raw)
        
#         t_icp_start = time.time()
#         self.publish_initial_pose(lx, ly, lyaw_rad, frame_id=lframe)

#         # Step D: MOTION FREEZE — Hold robot stationary while ICP calculates map -> odom transform
#         self.get_logger().info("⏳ Holding robot stationary for ICP point cloud registration lock...")
#         time.sleep(3.0) 

#         # Step E: Verify Costmap Readiness before sending motion goals
#         costmap_ready = self.wait_for_map_and_costmap_ready(timeout_sec=3.0)
        
#         t_map_ready_duration = time.time() - t_icp_start
#         total_transition_latency = time.time() - t_switch_start

#         if costmap_ready:
#             self.get_logger().info(
#                 f"⏱️ [METRIC] Map & Costmap ready for planning in: {t_map_ready_duration:.3f} seconds (post-initialpose)"
#             )
#         else:
#             self.get_logger().warn("⚠️ Costmap service timed out. Proceeding with goal attempt...")

#         self.get_logger().info(
#             f"⏱️ [METRIC] Total Switch-to-Ready Transition Latency: {total_transition_latency:.3f} seconds"
#         )

#         # STAGE 4: Execute Final Goal
#         self.get_logger().info(f"[STAGE 4] Navigating to final target on [{target_floor}]")
#         if self.send_nav_goal(target_x, target_y, yaw_rad=0.0, frame_id='map'):
#             goal_handle.succeed()
#             self.get_logger().info("🎉 Multi-floor mission completed successfully!")
#         else:
#             self.get_logger().error("Stage 4 final approach failed.")
#             goal_handle.abort()

#         return NavigateToPose.Result()


# def main():
#     rclpy.init()
#     node = MultiFloorDispatcherActionServer()

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
















###################################################################################
# import math
# import time
# import yaml
# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
# from rclpy.executors import MultiThreadedExecutor
# from rclpy.callback_groups import ReentrantCallbackGroup
# from rclpy.qos import (
#     QoSProfile,
#     QoSDurabilityPolicy,
#     QoSReliabilityPolicy,
#     QoSHistoryPolicy,
# )

# from std_msgs.msg import String
# from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
# from nav2_msgs.action import NavigateToPose
# from nav2_msgs.srv import GetCostmap
# from std_srvs.srv import SetBool


# class MultiFloorDispatcherActionServer(Node):

#     def __init__(self):
#         super().__init__('multi_floor_dispatcher_action_server')

#         self._cb_group = ReentrantCallbackGroup()

#         # Load configuration
#         self.declare_parameter(
#             'config_file',
#             '/home/nvidia/ros2_ws/src/whi_nav2_bringup/config/floor_config.yaml'
#         )
#         self.declare_parameter('default_floor', 'Floor_1')

#         config_path = self.get_parameter('config_file').get_parameter_value().string_value
#         self.active_floor = self.get_parameter('default_floor').get_parameter_value().string_value

#         with open(config_path, 'r') as f:
#             self.floor_config = yaml.safe_load(f)['floors']

#         self.get_logger().info(f"Loaded config for floors: {list(self.floor_config.keys())}")

#         # Nav2 Action Client
#         self._nav_client = ActionClient(
#             self, NavigateToPose, 'navigate_to_pose', callback_group=self._cb_group
#         )

#         # Latched QoS Profile
#         latch_qos = QoSProfile(
#             depth=1,
#             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
#             reliability=QoSReliabilityPolicy.RELIABLE,
#             history=QoSHistoryPolicy.KEEP_LAST,
#         )

#         # Publishers
#         self.floor_pub = self.create_publisher(String, 'current_floor', latch_qos)
#         self.elevator_call_pub = self.create_publisher(String, 'elevator/call_floor', 10)
#         self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
#         self.initial_pose_pub = self.create_publisher(
#             PoseWithCovarianceStamped, '/initialpose', 10
#         )

#         # Subscribers
#         self.floor_sub = self.create_subscription(
#             String, 'current_floor', self._current_floor_cb, 10, callback_group=self._cb_group
#         )
#         self.elevator_status_sub = self.create_subscription(
#             String, 'elevator/status', self._elevator_status_cb, 10, callback_group=self._cb_group
#         )

#         self.elevator_doors_open = False

#         # Service Clients
#         self.switch_db_cli = self.create_client(
#             SetBool, 'switch_floor_db', callback_group=self._cb_group
#         )
#         self.get_costmap_cli = self.create_client(
#             GetCostmap, '/global_costmap/get_costmap', callback_group=self._cb_group
#         )

#         # Action Server
#         self._action_server = ActionServer(
#             self,
#             NavigateToPose,
#             'navigate_multi_floor',
#             execute_callback=self.execute_action_callback,
#             goal_callback=self.goal_callback,
#             cancel_callback=self.cancel_callback,
#             callback_group=self._cb_group
#         )

#         self.get_logger().info(f"🚀 Action Server ready. Active floor: [{self.active_floor}]")

#     def _current_floor_cb(self, msg: String):
#         if msg.data in self.floor_config and msg.data != self.active_floor:
#             self.get_logger().info(f"Updated active floor state to: [{msg.data}]")
#             self.active_floor = msg.data

#     def _elevator_status_cb(self, msg: String):
#         if 'DOORS_OPEN' in msg.data:
#             self.elevator_doors_open = True

#     def goal_callback(self, goal_request):
#         self.get_logger().info("Received multi-floor navigation goal request.")
#         return GoalResponse.ACCEPT

#     def cancel_callback(self, goal_handle):
#         self.get_logger().warn("Goal cancel requested.")
#         return CancelResponse.ACCEPT

#     def parse_pose_config(self, pose_data):
#         if isinstance(pose_data, dict):
#             x = float(pose_data.get('x', 0.0))
#             y = float(pose_data.get('y', 0.0))
#             yaw = float(pose_data.get('yaw', 0.0))

#             # Auto-detect degrees if flag is set or magnitude > pi
#             if pose_data.get('use_degrees', False) or abs(yaw) > math.pi:
#                 yaw = math.radians(yaw)

#             frame_id = str(pose_data.get('frame_id', 'map'))
#         elif isinstance(pose_data, (list, tuple)):
#             x = float(pose_data[0])
#             y = float(pose_data[1])
#             yaw = float(pose_data[2]) if len(pose_data) > 2 else 0.0
#             if abs(yaw) > math.pi:
#                 yaw = math.radians(yaw)
#             frame_id = 'map'
#         else:
#             raise ValueError(f"Unsupported pose format in YAML: {type(pose_data)}")
#         return x, y, yaw, frame_id

#     def publish_initial_pose(self, x: float, y: float, yaw_rad: float = 0.0, frame_id: str = 'map'):
#         """Publishes initial pose seed to trigger ICP point cloud alignment."""
#         msg = PoseWithCovarianceStamped()
#         msg.header.frame_id = frame_id
#         msg.header.stamp = self.get_clock().now().to_msg()
#         msg.pose.pose.position.x = float(x)
#         msg.pose.pose.position.y = float(y)
#         msg.pose.pose.position.z = 0.0

#         msg.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
#         msg.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)

#         # Full 6x6 Covariance Matrix (Flattened to 36 elements)
#         cov = [0.0] * 36
#         cov[0] = 0.25   # X variance (0.5m std dev)
#         cov[7] = 0.25   # Y variance (0.5m std dev)
#         cov[14] = 1e-3  # Z variance
#         cov[21] = 1e-3  # Roll variance
#         cov[28] = 1e-3  # Pitch variance
#         cov[35] = 0.06  # Yaw variance (~14 deg std dev)
#         msg.pose.covariance = cov

#         # Burst publish 3 times to ensure delivery across ROS 2 network
#         for _ in range(3):
#             self.initial_pose_pub.publish(msg)
#             time.sleep(0.05)

#         self.get_logger().info(
#             f"📍 Published Initial Pose Seed: frame={frame_id}, x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw_rad):.1f}°"
#         )

#     def wait_for_map_and_costmap_ready(self, timeout_sec: float = 5.0) -> bool:
#         """Polls global costmap service to verify map readiness for path planning."""
#         if not self.get_costmap_cli.wait_for_service(timeout_sec=2.0):
#             self.get_logger().warn("Global costmap service unavailable, skipping polling.")
#             return False

#         req = GetCostmap.Request()
#         start_t = time.time()

#         while (time.time() - start_t) < timeout_sec:
#             future = self.get_costmap_cli.call_async(req)
#             while not future.done():
#                 time.sleep(0.05)

#             if future.result() is not None:
#                 return True
#             time.sleep(0.1)

#         return False

#     def send_nav_goal(self, x: float, y: float, yaw_rad: float = 0.0, frame_id: str = 'map') -> bool:
#         if not self._nav_client.wait_for_server(timeout_sec=5.0):
#             self.get_logger().error("Nav2 action server unavailable!")
#             return False

#         qz = math.sin(yaw_rad / 2.0)
#         qw = math.cos(yaw_rad / 2.0)

#         goal_msg = NavigateToPose.Goal()
#         goal_msg.pose.header.frame_id = frame_id
#         goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
#         goal_msg.pose.pose.position.x = float(x)
#         goal_msg.pose.pose.position.y = float(y)
#         goal_msg.pose.pose.orientation.z = qz
#         goal_msg.pose.pose.orientation.w = qw

#         self.get_logger().info(f"Dispatching Nav2 Goal -> frame: {frame_id}, x: {x:.2f}, y: {y:.2f}")

#         send_future = self._nav_client.send_goal_async(goal_msg)
#         while not send_future.done():
#             time.sleep(0.1)

#         goal_handle = send_future.result()
#         if not goal_handle.accepted:
#             self.get_logger().error("Goal rejected by Nav2!")
#             return False

#         result_future = goal_handle.get_result_async()
#         while not result_future.done():
#             time.sleep(0.1)

#         return result_future.result().status == 4  # SUCCEEDED

#     def drive_straight_blind(self, distance_m: float = 1.0, speed_mps: float = 0.15, direction_desc: str = "into elevator cabin"):
#         """Publishes direct velocity commands to move blindly forward on wheel odometry."""
#         self.get_logger().info(f"🤖 Driving blindly forward {distance_m:.2f}m at {speed_mps:.2f}m/s {direction_desc}...")

#         duration_sec = distance_m / speed_mps
#         rate_hz = 10
#         sleep_interval = 1.0 / rate_hz
#         steps = int(duration_sec * rate_hz)

#         twist = Twist()
#         twist.linear.x = float(speed_mps)

#         for _ in range(steps):
#             self.cmd_vel_pub.publish(twist)
#             time.sleep(sleep_interval)

#         # Stop the robot
#         stop_msg = Twist()
#         self.cmd_vel_pub.publish(stop_msg)
#         time.sleep(0.5)

#     def execute_action_callback(self, goal_handle):
#         req_pose = goal_handle.request.pose
#         target_x = req_pose.pose.position.x
#         target_y = req_pose.pose.position.y

#         req_frame = req_pose.header.frame_id
#         target_floor = req_frame if req_frame in self.floor_config else self.active_floor

#         self.get_logger().info(
#             f"🎯 Mission: Move from [{self.active_floor}] to [{target_floor}] at x={target_x:.2f}, y={target_y:.2f}"
#         )

#         # -------------------------------------------------------------
#         # CASE A: Same-Floor Navigation
#         # -------------------------------------------------------------
#         if self.active_floor == target_floor:
#             self.get_logger().info(f"Already on [{target_floor}]. Navigating directly to target...")
#             if self.send_nav_goal(target_x, target_y, frame_id='map'):
#                 goal_handle.succeed()
#                 return NavigateToPose.Result()
#             else:
#                 goal_handle.abort()
#                 return NavigateToPose.Result()

#         # -------------------------------------------------------------
#         # CASE B: Cross-Floor Navigation
#         # -------------------------------------------------------------
#         cur_floor_cfg = self.floor_config[self.active_floor]
#         target_floor_cfg = self.floor_config[target_floor]

#         # STAGE 1: Coarse Staging
#         coarse_raw = cur_floor_cfg['elevator_entry_pose']
#         cx, cy, cyaw_rad, cframe = self.parse_pose_config(coarse_raw)

#         self.get_logger().info(
#             f"[STAGE 1] Driving to staging pose (x={cx}, y={cy}, yaw={math.degrees(cyaw_rad):.1f}°) on [{self.active_floor}]"
#         )
#         if not self.send_nav_goal(cx, cy, cyaw_rad, frame_id=cframe):
#             self.get_logger().error("Stage 1 failed to reach elevator. Aborting mission.")
#             goal_handle.abort()
#             return NavigateToPose.Result()

#         # STAGE 2: Elevator Handshake & Parallel Database Switch
#         self.get_logger().info(f"[STAGE 2] Calling elevator to [{self.active_floor}]")
#         self.elevator_call_pub.publish(String(data=target_floor))

#         self.elevator_doors_open = False
#         start_time = time.time()

#         while not self.elevator_doors_open:
#             if goal_handle.is_cancel_requested:
#                 goal_handle.canceled()
#                 return NavigateToPose.Result()
#             if time.time() - start_time > 45.0:
#                 self.get_logger().error("Timed out waiting for elevator doors!")
#                 goal_handle.abort()
#                 return NavigateToPose.Result()
#             time.sleep(0.5)

#         # Doors Open -> Drive blindly in
#         entry_cfg = cur_floor_cfg.get('blind_entry', cur_floor_cfg.get('blind_exit', {}))
#         entry_dist = float(entry_cfg.get('distance', 1.0))
#         entry_speed = float(entry_cfg.get('speed', 0.15))
#         self.drive_straight_blind(distance_m=entry_dist, speed_mps=entry_speed, direction_desc="into cabin")

#         self.elevator_call_pub.publish(String(data=target_floor))

#         # Start timer for DB switch overhead
#         t_switch_start = time.time()

#         if not self.switch_db_cli.wait_for_service(timeout_sec=3.0):
#             self.get_logger().error("switch_floor_db service unavailable!")
#             goal_handle.abort()
#             return NavigateToPose.Result()

#         self.get_logger().info("⚡ Triggering background database switch during elevator transit...")
#         req = SetBool.Request()
#         req.data = (target_floor == "Floor_2")
#         db_switch_future = self.switch_db_cli.call_async(req)

#         # Wait for transit arrival
#         self.elevator_doors_open = False
#         start_time = time.time()

#         while not self.elevator_doors_open:
#             if goal_handle.is_cancel_requested:
#                 goal_handle.canceled()
#                 return NavigateToPose.Result()
#             if time.time() - start_time > 60.0:
#                 self.get_logger().error(f"Timed out waiting to arrive at [{target_floor}]!")
#                 goal_handle.abort()
#                 return NavigateToPose.Result()
#             time.sleep(0.5)

#         # STAGE 3: Exit Cabin & Verify DB Switch Latency
#         exit_cfg = target_floor_cfg.get('blind_exit', {})
#         exit_dist = float(exit_cfg.get('distance', 1.0))
#         exit_speed = float(exit_cfg.get('speed', 0.15))
#         self.drive_straight_blind(distance_m=exit_dist, speed_mps=exit_speed, direction_desc="out of cabin")

#         # Synchronize DB switch and record switch duration
#         while rclpy.ok() and not db_switch_future.done():
#             time.sleep(0.05)

#         db_switch_duration = time.time() - t_switch_start
#         self.get_logger().info(f"⏱️ [METRIC] Map DB Switch completed in: {db_switch_duration:.3f} seconds")

#         # Allow wheel odometry to settle
#         time.sleep(1.0)

#         # Seed Initial Pose
#         landing_raw = target_floor_cfg.get('landing_pose', target_floor_cfg.get('elevator_entry_pose'))
#         lx, ly, lyaw_rad, lframe = self.parse_pose_config(landing_raw)
        
#         t_icp_start = time.time()
#         self.publish_initial_pose(lx, ly, lyaw_rad, frame_id=lframe)

#         # Wait for ICP alignment + Costmap service readiness
#         self.get_logger().info("⏳ Waiting for ICP registration and Costmap update...")
#         time.sleep(2.0)  # Core ICP matching window

#         costmap_ready = self.wait_for_map_and_costmap_ready(timeout_sec=3.0)
        
#         t_map_ready_duration = time.time() - t_icp_start
#         total_transition_latency = time.time() - t_switch_start

#         if costmap_ready:
#             self.get_logger().info(
#                 f"⏱️ [METRIC] Map & Costmap ready for planning in: {t_map_ready_duration:.3f} seconds (post-initialpose)"
#             )
#         else:
#             self.get_logger().warn("⚠️ Costmap service timed out. Proceeding with goal attempt...")

#         self.get_logger().info(
#             f"⏱️ [METRIC] Total Switch-to-Ready Transition Latency: {total_transition_latency:.3f} seconds"
#         )

#         # STAGE 4: Execute Final Goal
#         self.get_logger().info(f"[STAGE 4] Navigating to final target on [{target_floor}]")
#         if self.send_nav_goal(target_x, target_y, yaw_rad=0.0, frame_id='map'):
#             goal_handle.succeed()
#             self.get_logger().info("🎉 Multi-floor mission completed successfully!")
#         else:
#             self.get_logger().error("Stage 4 final approach failed.")
#             goal_handle.abort()

#         return NavigateToPose.Result()


# def main():
#     rclpy.init()
#     node = MultiFloorDispatcherActionServer()

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



























################################################################################################


# import math
# import time
# import yaml
# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
# from rclpy.executors import MultiThreadedExecutor
# from rclpy.callback_groups import ReentrantCallbackGroup
# from rclpy.qos import (
#     QoSProfile,
#     QoSDurabilityPolicy,
#     QoSReliabilityPolicy,
#     QoSHistoryPolicy,
# )

# from std_msgs.msg import String
# from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
# from nav2_msgs.action import NavigateToPose
# from std_srvs.srv import SetBool


# class MultiFloorDispatcherActionServer(Node):

#     def __init__(self):
#         super().__init__('multi_floor_dispatcher_action_server')

#         self._cb_group = ReentrantCallbackGroup()

#         # Load configuration
#         self.declare_parameter(
#             'config_file',
#             '/home/nvidia/ros2_ws/src/whi_nav2_bringup/config/floor_config.yaml'
#         )
#         self.declare_parameter('default_floor', 'Floor_1')

#         config_path = self.get_parameter('config_file').get_parameter_value().string_value
#         self.active_floor = self.get_parameter('default_floor').get_parameter_value().string_value

#         with open(config_path, 'r') as f:
#             self.floor_config = yaml.safe_load(f)['floors']

#         self.get_logger().info(f"Loaded config for floors: {list(self.floor_config.keys())}")

#         # Nav2 Action Client
#         self._nav_client = ActionClient(
#             self, NavigateToPose, 'navigate_to_pose', callback_group=self._cb_group
#         )

#         # Latched QoS Profile matching rtabmap_db_switcher_node
#         latch_qos = QoSProfile(
#             depth=1,
#             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
#             reliability=QoSReliabilityPolicy.RELIABLE,
#             history=QoSHistoryPolicy.KEEP_LAST,
#         )

#         # Publishers
#         self.floor_pub = self.create_publisher(String, 'current_floor', latch_qos)
#         self.elevator_call_pub = self.create_publisher(String, 'elevator/call_floor', 10)
#         self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
#         self.initial_pose_pub = self.create_publisher(
#             PoseWithCovarianceStamped, '/initialpose', 10
#         )

#         # Subscribers
#         self.floor_sub = self.create_subscription(
#             String, 'current_floor', self._current_floor_cb, 10, callback_group=self._cb_group
#         )
#         self.elevator_status_sub = self.create_subscription(
#             String, 'elevator/status', self._elevator_status_cb, 10, callback_group=self._cb_group
#         )

#         self.elevator_doors_open = False

#         # Database Switcher Service Client
#         self.switch_db_cli = self.create_client(
#             SetBool, 'switch_floor_db', callback_group=self._cb_group
#         )

#         # Action Server
#         self._action_server = ActionServer(
#             self,
#             NavigateToPose,
#             'navigate_multi_floor',
#             execute_callback=self.execute_action_callback,
#             goal_callback=self.goal_callback,
#             cancel_callback=self.cancel_callback,
#             callback_group=self._cb_group
#         )

#         self.get_logger().info(f"🚀 Action Server ready. Active floor: [{self.active_floor}]")

#     def _current_floor_cb(self, msg: String):
#         if msg.data in self.floor_config and msg.data != self.active_floor:
#             self.get_logger().info(f"Updated active floor state to: [{msg.data}]")
#             self.active_floor = msg.data

#     def _elevator_status_cb(self, msg: String):
#         if 'DOORS_OPEN' in msg.data:
#             self.elevator_doors_open = True

#     def goal_callback(self, goal_request):
#         self.get_logger().info("Received multi-floor navigation goal request.")
#         return GoalResponse.ACCEPT

#     def cancel_callback(self, goal_handle):
#         self.get_logger().warn("Goal cancel requested.")
#         return CancelResponse.ACCEPT

#     def parse_pose_config(self, pose_data):
#         if isinstance(pose_data, dict):
#             x = float(pose_data.get('x', 0.0))
#             y = float(pose_data.get('y', 0.0))
#             yaw = float(pose_data.get('yaw', 0.0))

#             # Automatically convert if explicitly flagged OR if magnitude > pi
#             if pose_data.get('use_degrees', False) or abs(yaw) > math.pi:
#                 yaw = math.radians(yaw)

#             frame_id = str(pose_data.get('frame_id', 'map'))
#         elif isinstance(pose_data, (list, tuple)):
#             x = float(pose_data[0])
#             y = float(pose_data[1])
#             yaw = float(pose_data[2]) if len(pose_data) > 2 else 0.0
#             if abs(yaw) > math.pi:
#                 yaw = math.radians(yaw)
#             frame_id = 'map'
#         else:
#             raise ValueError(f"Unsupported pose format in YAML: {type(pose_data)}")
#         return x, y, yaw, frame_id

#     def publish_initial_pose(self, x: float, y: float, yaw_rad: float = 0.0, frame_id: str = 'map'):
#         """Publishes initial pose seed to trigger ICP point cloud alignment."""
#         msg = PoseWithCovarianceStamped()
#         msg.header.frame_id = frame_id
#         msg.header.stamp = self.get_clock().now().to_msg()
#         msg.pose.pose.position.x = float(x)
#         msg.pose.pose.position.y = float(y)
#         msg.pose.pose.position.z = 0.0

#         # Convert yaw (in radians) to quaternion orientation
#         msg.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
#         msg.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)

#         # Full 6x6 Covariance Matrix (Flattened 36-element array)
#         cov = [0.0] * 36
#         cov[0] = 0.25   # X variance (0.5m std dev)
#         cov[7] = 0.25   # Y variance (0.5m std dev)
#         cov[14] = 1e-3  # Z variance
#         cov[21] = 1e-3  # Roll variance
#         cov[28] = 1e-3  # Pitch variance
#         cov[35] = 0.06  # Yaw variance (~14 deg std dev)
#         msg.pose.covariance = cov

#         # Burst publish 3 times to ensure delivery across transient ROS 2 nodes
#         for _ in range(3):
#             self.initial_pose_pub.publish(msg)
#             time.sleep(0.05)

#         self.get_logger().info(
#             f"📍 Published Initial Pose Seed: frame={frame_id}, x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw_rad):.1f}° ({yaw_rad:.3f} rad)"
#         )

#     def send_nav_goal(self, x: float, y: float, yaw: float = 0.0, frame_id: str = 'map') -> bool:
#         if not self._nav_client.wait_for_server(timeout_sec=5.0):
#             self.get_logger().error("Nav2 action server unavailable!")
#             return False

#         yaw_rad = float(yaw)
#         qz = math.sin(yaw_rad / 2.0)
#         qw = math.cos(yaw_rad / 2.0)

#         goal_msg = NavigateToPose.Goal()
#         goal_msg.pose.header.frame_id = frame_id
#         goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
#         goal_msg.pose.pose.position.x = float(x)
#         goal_msg.pose.pose.position.y = float(y)
#         goal_msg.pose.pose.orientation.z = qz
#         goal_msg.pose.pose.orientation.w = qw

#         self.get_logger().info(f"Dispatching Nav2 Goal -> frame: {frame_id}, x: {x:.2f}, y: {y:.2f}")

#         send_future = self._nav_client.send_goal_async(goal_msg)
#         while not send_future.done():
#             time.sleep(0.1)

#         goal_handle = send_future.result()
#         if not goal_handle.accepted:
#             self.get_logger().error("Goal rejected by Nav2!")
#             return False

#         result_future = goal_handle.get_result_async()
#         while not result_future.done():
#             time.sleep(0.1)

#         return result_future.result().status == 4  # SUCCEEDED

#     def drive_straight_blind(self, distance_m: float = 1.0, speed_mps: float = 0.15, direction_desc: str = "into elevator cabin"):
#         """Publishes direct velocity commands to move blindly forward on wheel odometry."""
#         self.get_logger().info(f"🤖 Driving blindly forward {distance_m:.2f}m at {speed_mps:.2f}m/s {direction_desc}...")

#         duration_sec = distance_m / speed_mps
#         rate_hz = 10
#         sleep_interval = 1.0 / rate_hz
#         steps = int(duration_sec * rate_hz)

#         twist = Twist()
#         twist.linear.x = float(speed_mps)

#         for _ in range(steps):
#             self.cmd_vel_pub.publish(twist)
#             time.sleep(sleep_interval)

#         # Stop the robot
#         stop_msg = Twist()
#         self.cmd_vel_pub.publish(stop_msg)
#         time.sleep(0.5)

#     def execute_action_callback(self, goal_handle):
#         req_pose = goal_handle.request.pose
#         target_x = req_pose.pose.position.x
#         target_y = req_pose.pose.position.y

#         req_frame = req_pose.header.frame_id
#         if req_frame in self.floor_config:
#             target_floor = req_frame
#         else:
#             target_floor = self.active_floor

#         self.get_logger().info(
#             f"🎯 Mission: Move from [{self.active_floor}] to [{target_floor}] at x={target_x:.2f}, y={target_y:.2f}"
#         )

#         # -------------------------------------------------------------
#         # CASE A: Same-Floor Navigation
#         # -------------------------------------------------------------
#         if self.active_floor == target_floor:
#             self.get_logger().info(f"Already on [{target_floor}]. Navigating directly to target...")
#             if self.send_nav_goal(target_x, target_y, frame_id='map'):
#                 goal_handle.succeed()
#                 return NavigateToPose.Result()
#             else:
#                 goal_handle.abort()
#                 return NavigateToPose.Result()

#         # -------------------------------------------------------------
#         # CASE B: Cross-Floor Navigation
#         # -------------------------------------------------------------
#         cur_floor_cfg = self.floor_config[self.active_floor]
#         target_floor_cfg = self.floor_config[target_floor]

#         # STAGE 1: Drive to coarse staging pose on CURRENT floor
#         coarse_raw = cur_floor_cfg['elevator_entry_pose']
#         cx, cy, cyaw, cframe = self.parse_pose_config(coarse_raw)

#         self.get_logger().info(
#             f"[STAGE 1] Driving to coarse staging pose (x={cx}, y={cy}) on current floor [{self.active_floor}]"
#         )
#         if not self.send_nav_goal(cx, cy, cyaw, frame_id=cframe):
#             self.get_logger().error("Stage 1 failed to reach elevator. Aborting mission.")
#             goal_handle.abort()
#             return NavigateToPose.Result()

#         # STAGE 2: ELEVATOR HANDSHAKE & PARALLEL TRANSIT
#         self.get_logger().info(f"[STAGE 2] Calling elevator to [{self.active_floor}]")
#         self.elevator_call_pub.publish(String(data=target_floor))

#         self.elevator_doors_open = False
#         start_time = time.time()
#         timeout_sec = 45.0

#         while not self.elevator_doors_open:
#             if goal_handle.is_cancel_requested:
#                 goal_handle.canceled()
#                 return NavigateToPose.Result()

#             if time.time() - start_time > timeout_sec:
#                 self.get_logger().error("Timed out waiting for elevator doors!")
#                 goal_handle.abort()
#                 return NavigateToPose.Result()

#             time.sleep(0.5)

#         # Doors Open -> Drive blindly into cabin using blind_entry (or blind_exit defaults)
#         self.get_logger().info("[STAGE 2] Elevator doors open!")
#         entry_cfg = cur_floor_cfg.get('blind_entry', cur_floor_cfg.get('blind_exit', {}))
#         entry_dist = float(entry_cfg.get('distance', 1.0))
#         entry_speed = float(entry_cfg.get('speed', 0.15))

#         self.drive_straight_blind(distance_m=entry_dist, speed_mps=entry_speed, direction_desc="into elevator cabin")

#         # Command elevator to travel to target floor
#         self.get_logger().info(f"[STAGE 2] Inside cabin. Requesting transit to [{target_floor}]...")
#         self.elevator_call_pub.publish(String(data=target_floor))

#         # TRIGGER PARALLEL DB SWITCH WHILE ELEVATOR IS MOVING
#         if not self.switch_db_cli.wait_for_service(timeout_sec=3.0):
#             self.get_logger().error("switch_floor_db service unavailable!")
#             goal_handle.abort()
#             return NavigateToPose.Result()

#         self.get_logger().info("⚡ Triggering background database switch during elevator transit...")
        
#         req = SetBool.Request()
#         req.data = (target_floor == "Floor_2")
#         db_switch_future = self.switch_db_cli.call_async(req)

#         # Wait for elevator to reach target floor while DB loads in parallel
#         self.elevator_doors_open = False
#         start_time = time.time()
#         timeout_sec = 60.0

#         while not self.elevator_doors_open:
#             if goal_handle.is_cancel_requested:
#                 goal_handle.canceled()
#                 return NavigateToPose.Result()

#             if time.time() - start_time > timeout_sec:
#                 self.get_logger().error(f"Timed out waiting to arrive at [{target_floor}]!")
#                 goal_handle.abort()
#                 return NavigateToPose.Result()

#             time.sleep(0.5)

#         # STAGE 3: EXIT CABIN, SYNC DB SWITCH & SEED INITIAL POSE
#         self.get_logger().info(f"[STAGE 3] Arrived at [{target_floor}]. Exiting cabin blindly...")
#         exit_cfg = target_floor_cfg.get('blind_exit', {})
#         exit_dist = float(exit_cfg.get('distance', 1.0))
#         exit_speed = float(exit_cfg.get('speed', 0.15))

#         self.drive_straight_blind(distance_m=exit_dist, speed_mps=exit_speed, direction_desc="out of elevator cabin")

#         # Synchronize with background DB switch task
#         self.get_logger().info("[STAGE 3] Synchronizing with database switcher node...")
#         while rclpy.ok() and not db_switch_future.done():
#             time.sleep(0.1)

#         res = db_switch_future.result()
#         if not res or not res.success:
#             self.get_logger().error(f"Failed to switch database: {res.message if res else 'No response'}")
#             goal_handle.abort()
#             return NavigateToPose.Result()

#         self.get_logger().info("[STAGE 3] Map database switch verified!")

#         # Seed initial pose at landing coordinates for ICP refinement (falls back to elevator_entry_pose)
#         landing_raw = target_floor_cfg.get('landing_pose', target_floor_cfg.get('elevator_entry_pose'))
#         lx, ly, lyaw, lframe = self.parse_pose_config(landing_raw)
#         self.publish_initial_pose(lx, ly, lyaw, frame_id=lframe)

#         # Pause to let ICP lock onto LiDAR scan and refine TF
#         self.get_logger().info("⏳ Waiting 2s for LiDAR ICP point cloud matching to align...")
#         time.sleep(5.0)

#         # STAGE 4: Final Goal Execution on NEW floor
#         self.get_logger().info(f"[STAGE 4] Navigating to final target on [{target_floor}]")
#         if self.send_nav_goal(target_x, target_y, frame_id='map'):
#             goal_handle.succeed()
#             self.get_logger().info("🎉 Multi-floor mission completed successfully!")
#         else:
#             self.get_logger().error("Stage 4 final approach failed.")
#             goal_handle.abort()

#         return NavigateToPose.Result()


# def main():
#     rclpy.init()
#     node = MultiFloorDispatcherActionServer()

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
