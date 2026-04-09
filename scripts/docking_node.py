#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from transforms3d.euler import quat2euler, euler2quat
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Bool
from nav_msgs.msg import Path
from std_srvs.srv import SetBool
import open3d as o3d
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory
from enum import Enum
import threading
import copy

class ActiveState(Enum):
    INACTIVE = 0
    DOCKING = 1
    UNDOCKING = 2


class LidarDockingNode(Node):
    def __init__(self):
        super().__init__('lidar_docking')

        # --- params ---
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('target_frame', 'odom')
        self.declare_parameter('template_file', 'dock_template_sym.pcd')
        self.declare_parameter('template_normal_angle', 90.0)
        self.declare_parameter('template_side_length', 0.2)
        self.declare_parameter('downsample_size', 0.01)
        self.declare_parameter('debug.visualize', False)
        self.base_frame = self.get_parameter('base_frame').value
        self.target_frame = self.get_parameter('target_frame').value
        self.template_file = self.get_parameter('template_file').value
        self.template_normal_angle = self.get_parameter('template_normal_angle').value
        self.template_side_length = self.get_parameter('template_side_length').value
        self.downsample_size = self.get_parameter('downsample_size').value
        self.debug_visualize = self.get_parameter('debug.visualize').value

        # --- TF setup ---
        self.tf2_buffer = Buffer()
        self.tf2_listener = TransformListener(self.tf2_buffer, self)

        # --- Variables ---
        self.undock_pose = PoseStamped()
        
        # --- 1. State Variables ---
        self.active = ActiveState.INACTIVE
        self.last_T = np.identity(4)
        self.is_locked = False
        self.template_ready = False

        # --- 2. Load and Prepare Template ---
        try:
            pkg_path = get_package_share_directory('whi_nav2_bringup')
            pcd_path = os.path.join(pkg_path, 'data', self.template_file)
            self.template_pcd = o3d.io.read_point_cloud(pcd_path)
            
            if self.template_pcd.is_empty():
                self.get_logger().error(f"Template PCD is empty! Path: {pcd_path}")
            else:
                self.template_pcd = self.template_pcd.voxel_down_sample(self.downsample_size)
                pcd_downsampled = copy.deepcopy(self.template_pcd)
                points = np.asarray(self.template_pcd.points)[:, :2]
                # find the possible lines
                min_inliers = max(3, int(0.85 * self.template_side_length / self.downsample_size))
                lines, _ = self.extract_lines_ransac_2d(
                    points,
                    max_lines=3,
                    threshold=0.003,
                    min_inliers=min_inliers)
                # filter lines with certain intersected angle
                pair = self.select_line_pair_by_angle(
                    lines,
                    target_angle=self.template_normal_angle,
                    angle_tolerance=10.0)
                if pair is not None:
                    # calculate the intersection point for translation
                    intersection = self.intersect_lines_2d(pair[0], pair[1])
                    translation = np.array([intersection[0], intersection[1], 0.0])
                else:
                    translation = self.template_pcd.get_center()
                    self.get_logger().error("No valid line pair found!")

                self.template_pcd.translate(-translation)
                self.template_ready = True
                self.get_logger().info(f"Template Ready. Points: {len(self.template_pcd.points)}")

                # visualize the result
                if self.debug_visualize:
                    line_set = self.lines_to_lineset(pair, length=0.2)
                    self.visualize_pcds(
                        [pcd_downsampled, self.template_pcd, line_set],
                        [[1, 0, 0], [0, 1, 0]],  # only applies to pcd
                        False
                    )

        except Exception as e:
            self.get_logger().error(f"Failed to load PCD: {str(e)}")

        # --- 3. ROS Communication ---
        self.docking_trigger_sub = self.create_subscription(Bool, 'docking_trigger', self.docking_trigger_sub_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.pose_pub = self.create_publisher(PoseStamped, 'detected_dynamic_pose', 10)
        self.path_pub = self.create_publisher(Path, 'docking_visual_path', 10)
        # trigger docking
        self.srv = self.create_service(SetBool, 'docking_trigger', self.docking_trigger_callback)
        
        self.get_logger().info("Docking Node Ready. Waiting for trigger...")

    def get_robot_pose_in_frame(self, frame: str) -> PoseStamped:
        robot_pose = PoseStamped()
        
        # Set source frame
        robot_pose.header.frame_id = self.base_frame
        robot_pose.header.stamp = Time(seconds=0).to_msg()  # equivalent to rclcpp::Time(0)

        # Transform pose into target frame
        return self.tf2_buffer.transform(robot_pose, frame)

    def compute_undock_pose(self, x_offset, yaw_offset) -> PoseStamped:
        # based on robot's current pose
        robot_pose = self.get_robot_pose_in_frame(self.target_frame)

        # --- get yaw from quaternion ---
        quat = [robot_pose.pose.orientation.w,
            robot_pose.pose.orientation.x, robot_pose.pose.orientation.y, robot_pose.pose.orientation.z]
        roll, pitch, yaw = quat2euler(quat)

        # --- create PoseStamped ---
        undock_pose = PoseStamped()
        undock_pose.header.frame_id = robot_pose.header.frame_id
        undock_pose.header.stamp = self.get_clock().now().to_msg()

        # --- apply translation offset ---
        undock_pose.pose.position.x = robot_pose.pose.position.x + math.cos(yaw) * x_offset
        undock_pose.pose.position.y = robot_pose.pose.position.y + math.sin(yaw) * x_offset
        undock_pose.pose.position.z = robot_pose.pose.position.z

        # --- apply yaw offset ---
        q_new = euler2quat(0.0, 0.0, yaw + yaw_offset)

        undock_pose.pose.orientation.x = q_new[1]
        undock_pose.pose.orientation.y = q_new[2]
        undock_pose.pose.orientation.z = q_new[3]
        undock_pose.pose.orientation.w = q_new[0]

        return undock_pose

    def docking_trigger_sub_callback(self, msg):
        self.active = ActiveState.DOCKING if msg.data else ActiveState.INACTIVE
        if self.active == ActiveState.DOCKING:
            self.get_logger().info(">>> ICP DOCKING ACTIVATED")
        else:
            # calculate the undock pose
            # Make sure that the undock pose is pointing in the same direction when moving backwards
            self.undock_pose = self.compute_undock_pose(-1.0, math.pi)
            self.get_logger().info("<<< ICP DOCKING DEACTIVATED")
            self.reset_state("Deactivated")

    def docking_trigger_callback(self, request, response):
        self.active = ActiveState.DOCKING if request.data else ActiveState.INACTIVE
        if self.active == ActiveState.DOCKING:
            self.get_logger().info(">>> ICP DOCKING ACTIVATED")
        else:
            # calculate the undock pose
            # Make sure that the undock pose is pointing in the same direction when moving backwards
            self.undock_pose = self.compute_undock_pose(-1.0, math.pi)
            self.get_logger().info("<<< ICP DOCKING DEACTIVATED")
            self.reset_state("Deactivated")

        response.success = True
        return response

    def scan_callback(self, msg):
        if not self.template_ready or self.active == ActiveState.INACTIVE:
            return
        elif self.active == ActiveState.UNDOCKING:
            self.undock_pose.header.stamp = msg.header.stamp
            self.pose_pub.publish(self.undock_pose)
            return

        # --- Step A: Process Scan Data ---
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        
        # Filter: 2m range, 60-degree cone in front
        mask = (np.abs(angles) < np.radians(60)) & (ranges < 2.0) & (ranges > 0.05)
        
        if np.sum(mask) < 15:
            if self.is_locked:
                self.reset_state("Lost visibility of dock")
            return

        x = ranges[mask] * np.cos(angles[mask])
        y = ranges[mask] * np.sin(angles[mask])
        points = np.vstack((x, y, np.zeros_like(x))).T

        live_pcd = o3d.geometry.PointCloud()
        live_pcd.points = o3d.utility.Vector3dVector(points)
        live_pcd = live_pcd.voxel_down_sample(self.downsample_size)

        # --- Step B: Dynamic ICP Search ---
        # Tighten search distance once locked to prevent jumping to wrong side of dock
        search_dist = 0.4 if self.is_locked else 1.2 
        
        criteria = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100)

        reg_p2p = o3d.pipelines.registration.registration_icp(
            self.template_pcd, 
            live_pcd, 
            search_dist, 
            self.last_T,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            criteria
        )

        T = reg_p2p.transformation
        yaw = np.arctan2(T[1, 0], T[0, 0])

        # --- Step C: HEADING GUARD (Anti-Flip Logic) ---
        # Since the LIDAR looks forward, the dock face should point toward the robot.
        # If yaw is > 90deg, it's mathematically flipped 180deg.
        if abs(yaw) > (np.pi / 2.0):
            self.get_logger().warn("180° Flip Detected! Correcting Heading...")
            # Create a 180-degree Z-rotation matrix
            flip_z = np.array([
                [-1,  0,  0,  0],
                [ 0, -1,  0,  0],
                [ 0,  0,  1,  0],
                [ 0,  0,  0,  1]
            ])
            T = T @ flip_z
            yaw = np.arctan2(T[1, 0], T[0, 0])

        # --- Step D: Validation & Publishing ---
        # Stricter thresholds for RMSE to ensure stability
        if reg_p2p.fitness > 0.70 and reg_p2p.inlier_rmse < 0.06:
            self.is_locked = True
            self.last_T = T # Save corrected T for next iteration seed
            
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(T[0, 3])
            pose.pose.position.y = float(T[1, 3]) #+ 0.01 #0.04 # Your manual offset
            
            pose.pose.orientation.z = np.sin(yaw / 2.0)
            pose.pose.orientation.w = np.cos(yaw / 2.0)
            
            self.pose_pub.publish(pose)
            self.publish_visual_path(msg.header, pose)

            # self.get_logger().info(
            #             f"LOCKED! Dist: {pose.pose.position.x:.2f}m | "
            #             f"Theta: {np.degrees(yaw):.1f}° | "
            #             f"RMSE: {reg_p2p.inlier_rmse:.4f}",
            #             throttle_duration_sec=0.5
            # )

            # self.get_logger().info(
            #     f"LOCKED! Dist: {pose.pose.position.x:.2f}m | RMSE: {reg_p2p.inlier_rmse:.4f}",
            #     throttle_duration_sec=0.5
            # )
        else:
            # If tracking is bad, don't update last_T with a bad pose
            if reg_p2p.fitness < 0.2:
                self.reset_state("Poor match fitness")

    def publish_visual_path(self, header, pose):
        path_msg = Path()
        path_msg.header = header
        start_point = PoseStamped()
        start_point.header = header
        path_msg.poses.append(start_point)
        path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)

    def reset_state(self, reason):
        if self.is_locked:
            self.get_logger().warn(f"DOCK LOST: {reason}")
        self.is_locked = False
        self.last_T = np.identity(4)

    # lines
    def extract_lines_ransac_2d(self, points, max_lines=3, threshold=0.02, min_inliers=20, iterations=500):
        """
        Extract multiple 2D lines from point cloud using RANSAC.

        :param points: Nx2 numpy array
        :param max_lines: maximum number of lines to extract
        :param threshold: distance threshold for inliers
        :param min_inliers: minimum points to accept a line
        :param iterations: RANSAC iterations
        :return: list of (point_on_line, direction), list of inlier point sets
        """

        def fit_line_ransac_2d(points):
            best_inliers_mask = None
            best_model = None
            best_count = 0

            for _ in range(iterations):
                if len(points) < 2:
                    break

                i, j = np.random.choice(len(points), 2, replace=False)
                p1, p2 = points[i], points[j]

                direction = p2 - p1
                norm = np.linalg.norm(direction)
                if norm == 0:
                    continue
                direction /= norm

                diff = points - p1
                dist = np.abs(diff[:, 0]*direction[1] - diff[:, 1]*direction[0])

                inliers_mask = dist < threshold
                count = np.sum(inliers_mask)

                if count > best_count:
                    best_count = count
                    best_inliers_mask = inliers_mask
                    best_model = (p1, direction)

            return best_model, best_inliers_mask

        remaining = points.copy()
        lines = []
        line_inliers = []

        for _ in range(max_lines):
            model, inliers_mask = fit_line_ransac_2d(remaining)

            if model is None or inliers_mask is None:
                break

            inliers = remaining[inliers_mask]

            if len(inliers) < min_inliers:
                break

            lines.append(model)
            line_inliers.append(inliers)

            # Remove inliers efficiently
            remaining = remaining[~inliers_mask]

            if len(remaining) < min_inliers:
                break

        return lines, line_inliers

    def select_line_pair_by_angle(self, lines, target_angle=90.0, angle_tolerance=1.0):
        """
        Select a pair of lines whose angle is closest to target_angle.

        :param lines: list of (point, direction)
        :param target_angle: desired angle between lines (degrees)
        :param angle_tolerance: allowed deviation (degrees)
        :return: (line1, line2) or None
        """

        def angle_between(d1, d2):
            cos_theta = np.dot(d1, d2)
            return np.arccos(np.clip(np.abs(cos_theta), -1.0, 1.0))

        best_pair = None
        best_error = float("inf")

        for i in range(len(lines)):
            for j in range(i + 1, len(lines)):
                d1 = lines[i][1]
                d2 = lines[j][1]

                angle = angle_between(d1, d2)
                error = abs(angle - math.radians(target_angle))

                if error < math.radians(angle_tolerance) and error < best_error:
                    best_error = error
                    best_pair = (lines[i], lines[j])

        return best_pair

    def intersect_lines_2d(self, line1, line2):
        """
        line = (point, direction)
        """
        p1, d1 = line1
        p2, d2 = line2

        # Solve: p1 + t*d1 = p2 + s*d2
        A = np.array([d1, -d2]).T  # 2x2
        b = p2 - p1

        try:
            t = np.linalg.solve(A, b)
            intersection = p1 + t[0] * d1
            return intersection
        except np.linalg.LinAlgError:
            return None

    # debug utilities
    def lines_to_lineset(self, lines, length=1.0):
        """
        Convert RANSAC lines into Open3D LineSet.

        :param lines: list of (point, direction)
        :param length: half length of each visualized line
        :return: LineSet
        """
        line_points = []
        line_indices = []

        for i, (p, d) in enumerate(lines):
            p1 = p - d * length
            p2 = p + d * length

            # convert to 3D (z=0)
            line_points.append([p1[0], p1[1], 0])
            line_points.append([p2[0], p2[1], 0])

            line_indices.append([2*i, 2*i+1])

        line_set = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(line_points),
            lines=o3d.utility.Vector2iVector(line_indices)
        )

        return line_set

    def visualize_pcds(self, pcd_list, colors=None, show_frame=True, window_name="Open3D Viewer"):
        """
        Visualize multiple point clouds.

        :param pcd_list: list of open3d.geometry.PointCloud
        :param colors: optional list of [r,g,b] colors (same length as pcd_list)
        :param show_frame: whether to show coordinate frame
        :param window_name: window title
        """

        geometries = []

        for i, geom in enumerate(pcd_list):
            if geom is None:
                continue

            if isinstance(geom, o3d.geometry.PointCloud):
                if colors and i < len(colors):
                    geom_copy = copy.deepcopy(geom)
                    geom_copy.paint_uniform_color(colors[i])
                    geometries.append(geom_copy)
                else:
                    geometries.append(geom)
            else:
                # LineSet / Mesh → just add directly
                geometries.append(geom)

        # Optional coordinate frame
        if show_frame:
            frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
            geometries.append(frame)

        # Run in separate thread (non-blocking)
        threading.Thread(
            target=o3d.visualization.draw_geometries,
            args=(geometries,),
            kwargs={"window_name": window_name},
            daemon=True
        ).start()


def main():
    rclpy.init()
    node = LidarDockingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
