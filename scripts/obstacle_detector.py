#!/usr/bin/env python3

############## This script does not use RGB Image but works fine with slope detection and obstacle detection (minor changes required for obstacle)
# import math
# import numpy as np
# import cv2

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image, LaserScan
# from std_msgs.msg import Bool
# from cv_bridge import CvBridge


# class HighFreqDepthObstacleDetector(Node):
#     def __init__(self):
#         super().__init__('high_freq_depth_obstacle_detector')

#         self.bridge = CvBridge()

#         # --- TOPICS CONFIGURATION ---
#         self.input_depth_topic = '/camera/camera/aligned_depth_to_color/image_raw'

#         self.output_obstacles_65_topic = '/detected_obstacles_65'
#         self.output_obstacles_75_topic = '/detected_obstacles_75'

#         self.output_slope_topic = '/obstacle/slope_detected'
#         self.output_moderate_slope_topic = '/obstacle/moderate_slope_detected'

#         # --- PARAMETERS ---
#         self.declare_parameter('row1_pct', 0.65)
#         self.declare_parameter('row2_pct', 0.75)
#         self.declare_parameter('min_slope_threshold_deg', 12.0)
#         self.declare_parameter('max_slope_threshold_deg', 17.0)

#         self.row1_pct = self.get_parameter('row1_pct').get_parameter_value().double_value
#         self.row2_pct = self.get_parameter('row2_pct').get_parameter_value().double_value
#         self.min_slope_deg = self.get_parameter('min_slope_threshold_deg').get_parameter_value().double_value
#         self.max_slope_deg = self.get_parameter('max_slope_threshold_deg').get_parameter_value().double_value

#         # Motion & Sensor Config
#         self.obstacle_height_thresh = 0.08  # 8 cm height off fitted plane
#         self.ditch_depth_thresh = 0.12     # 12 cm drop below fitted plane
#         self.max_detection_range = 3.00    # 3.0 m max range
#         self.hfov_rad = 1.50               # ~86 deg
#         self.v_window = 5

#         # Tightened Spatial & Temporal Filters (Fast response)
#         self.min_cluster_width = 3
#         self.history_depth = 2

#         # Calibration State
#         self.calibrated = False
#         self.calibration_frames_needed = 20
#         self.baseline_normal_samples = []
#         self.baseline_floor_normal = None

#         # Temporal History Buffers
#         self.r65_consecutive_counts = None
#         self.r75_consecutive_counts = None

#         # Cache for Decoupled Timer Publishing
#         self.latest_scan_65 = None
#         self.latest_scan_75 = None
#         self.latest_slope_flag = False
#         self.latest_mod_slope_flag = False

#         # --- DIRECT DEPTH SUBSCRIPTION (No RGB sync lag) ---
#         self.depth_sub = self.create_subscription(
#             Image,
#             self.input_depth_topic,
#             self.depth_callback,
#             10
#         )

#         # --- PUBLISHERS ---
#         self.pub_obs_65 = self.create_publisher(LaserScan, self.output_obstacles_65_topic, 10)
#         self.pub_obs_75 = self.create_publisher(LaserScan, self.output_obstacles_75_topic, 10)
#         self.pub_slope = self.create_publisher(Bool, self.output_slope_topic, 10)
#         self.pub_mod_slope = self.create_publisher(Bool, self.output_moderate_slope_topic, 10)

#         # --- HIGH-FREQUENCY PUBLISH TIMER (30 Hz) ---
#         self.publish_rate_hz = 30.0
#         self.create_timer(1.0 / self.publish_rate_hz, self.timer_publish_callback)

#         self.get_logger().info(
#             f"High-Frequency 30Hz Ground Monitor Initialized.\n"
#             f"Slope Navigability Window: [{self.min_slope_deg}° - {self.max_slope_deg}°]"
#         )

#     # --------------------------------------------------------------------------
#     # Helper: Extract Clean 1D Depth Profile
#     # --------------------------------------------------------------------------
#     def get_guided_row_profile(self, depth_matrix, center_row_idx, width):
#         half_w = self.v_window // 2
#         start_row = max(0, center_row_idx - half_w)
#         end_row = min(depth_matrix.shape[0], center_row_idx + half_w + 1)

#         depth_block = depth_matrix[start_row:end_row, :]
#         valid_mask = (depth_block > 0.1) & (depth_block < self.max_detection_range)

#         with np.errstate(all='ignore'):
#             block_with_nans = np.where(valid_mask, depth_block, np.nan)
#             all_nan_cols = np.all(np.isnan(block_with_nans), axis=0)
#             composite_row = np.nanmedian(block_with_nans, axis=0)
#             composite_row[all_nan_cols] = 0.0

#         return np.nan_to_num(composite_row, nan=0.0)

#     # --------------------------------------------------------------------------
#     # Helper: Spatial Cluster Filter
#     # --------------------------------------------------------------------------
#     def filter_isolated_noise(self, hazard_mask):
#         kernel = np.ones((1, self.min_cluster_width), dtype=np.uint8)
#         cleaned_mask = cv2.morphologyEx(hazard_mask.astype(np.uint8), cv2.MORPH_OPEN, kernel)
#         return cleaned_mask.astype(bool)

#     # --------------------------------------------------------------------------
#     # Helper: Vectorized 3D RANSAC Plane Fitting (Downsampled for speed)
#     # --------------------------------------------------------------------------
#     def fit_ransac_plane(self, depth_meters, row_start_pct=0.60, row_end_pct=0.90, sample_stride=5):
#         height, width = depth_meters.shape
#         start_row = int(height * row_start_pct)
#         end_row = int(height * row_end_pct)

#         v_idx, u_idx = np.mgrid[start_row:end_row:sample_stride, 0:width:sample_stride]
#         z = depth_meters[start_row:end_row:sample_stride, 0:width:sample_stride]

#         valid_mask = (z > 0.3) & (z < self.max_detection_range)
#         num_valid = np.sum(valid_mask)
#         if num_valid < 40:
#             return None, 0, None

#         cx = width / 2.0
#         cy = height / 2.0
#         focal_length = cx / math.tan(self.hfov_rad / 2.0)

#         z_valid = z[valid_mask]
#         u_valid = u_idx[valid_mask]
#         v_valid = v_idx[valid_mask]

#         x_valid = (u_valid - cx) * z_valid / focal_length
#         y_valid = (v_valid - cy) * z_valid / focal_length

#         points = np.column_stack((x_valid, y_valid, z_valid))

#         max_iterations = 30
#         distance_threshold = 0.03

#         idx = np.random.randint(0, num_valid, size=(max_iterations, 3))
#         p1 = points[idx[:, 0]]
#         p2 = points[idx[:, 1]]
#         p3 = points[idx[:, 2]]

#         v1 = p2 - p1
#         v2 = p3 - p1
#         normals = np.cross(v1, v2)
#         norms = np.linalg.norm(normals, axis=1, keepdims=True)

#         valid_normals = (norms.squeeze() > 1e-6)
#         if not np.any(valid_normals):
#             return None, 0, None

#         normals = np.divide(normals, norms, out=np.zeros_like(normals), where=norms > 1e-6)

#         d = -np.einsum('ij,ij->i', normals, p1)
#         dist_matrix = np.abs(np.dot(normals, points.T) + d[:, np.newaxis])

#         inliers_counts = np.sum(dist_matrix < distance_threshold, axis=1)
#         best_idx = np.argmax(inliers_counts)

#         best_inliers_count = int(inliers_counts[best_idx])
#         best_normal = normals[best_idx]
#         best_d = d[best_idx]

#         if best_normal is not None and best_normal[1] > 0:
#             best_normal = -best_normal
#             best_d = -best_d

#         plane_eq = (best_normal[0], best_normal[1], best_normal[2], best_d)
#         return best_normal, best_inliers_count, plane_eq

#     # --------------------------------------------------------------------------
#     # Direct Depth Callback
#     # --------------------------------------------------------------------------
#     def depth_callback(self, depth_msg: Image):
#         try:
#             cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
#         except Exception as e:
#             self.get_logger().error(f"Failed to parse depth frame: {str(e)}")
#             return

#         if cv_depth.dtype == np.uint16:
#             depth_meter = cv_depth.astype(np.float32) / 1000.0
#         else:
#             depth_meter = cv_depth.astype(np.float32)

#         height, width = depth_meter.shape

#         if self.r65_consecutive_counts is None:
#             self.r65_consecutive_counts = np.zeros(width, dtype=np.int32)
#             self.r75_consecutive_counts = np.zeros(width, dtype=np.int32)

#         row1_idx = int(height * self.row1_pct)
#         row2_idx = int(height * self.row2_pct)

#         row1_data = self.get_guided_row_profile(depth_meter, row1_idx, width)
#         row2_data = self.get_guided_row_profile(depth_meter, row2_idx, width)

#         current_normal, inlier_count, plane_eq = self.fit_ransac_plane(depth_meter)

#         # --- CALIBRATION PHASE ---
#         if not self.calibrated:
#             if current_normal is not None and inlier_count > 150:
#                 self.baseline_normal_samples.append(current_normal)

#             if len(self.baseline_normal_samples) >= self.calibration_frames_needed:
#                 avg_normal = np.median(self.baseline_normal_samples, axis=0)
#                 self.baseline_floor_normal = avg_normal / np.linalg.norm(avg_normal)
#                 self.calibrated = True
#                 self.get_logger().info("--> CALIBRATION COMPLETE! Ground plane baseline registered.")
#             return

#         # --- SLOPE EVALUATION ---
#         slope_detected_flag = False
#         moderate_slope_flag = False

#         if current_normal is not None and inlier_count > 100:
#             dot_product = np.clip(np.dot(current_normal, self.baseline_floor_normal), -1.0, 1.0)
#             angle_diff_deg = math.degrees(math.acos(dot_product))

#             if self.min_slope_deg <= angle_diff_deg <= self.max_slope_deg:
#                 slope_detected_flag = True

#             if self.min_slope_deg <= angle_diff_deg <= 15.0:
#                 moderate_slope_flag = True

#         self.latest_slope_flag = slope_detected_flag
#         self.latest_mod_slope_flag = moderate_slope_flag

#         # --- PLANE-RELATIVE HAZARD DETECTION ---
#         r65_hazards = np.zeros(width, dtype=bool)
#         r75_hazards = np.zeros(width, dtype=bool)

#         cx = width / 2.0
#         cy = height / 2.0
#         focal_length = cx / math.tan(self.hfov_rad / 2.0)

#         if plane_eq is not None:
#             A, B, C, D = plane_eq

#             u_indices = np.arange(width)
#             x_dir = (u_indices - cx) / focal_length

#             # Row 65% projected floor depth
#             y_dir_1 = (row1_idx - cy) / focal_length
#             denom_1 = (A * x_dir + B * y_dir_1 + C)
#             expected_z1 = np.where(np.abs(denom_1) > 1e-5, -D / denom_1, 0.0)

#             # Row 75% projected floor depth
#             y_dir_2 = (row2_idx - cy) / focal_length
#             denom_2 = (A * x_dir + B * y_dir_2 + C)
#             expected_z2 = np.where(np.abs(denom_2) > 1e-5, -D / denom_2, 0.0)

#             for i in range(width):
#                 # Row 65% check
#                 z1_actual = row1_data[i]
#                 z1_exp = expected_z1[i]
#                 if z1_actual > 0.1 and z1_exp > 0.1:
#                     diff = z1_exp - z1_actual
#                     if diff > self.obstacle_height_thresh or diff < -self.ditch_depth_thresh:
#                         r65_hazards[i] = True

#                 # Row 75% check
#                 z2_actual = row2_data[i]
#                 z2_exp = expected_z2[i]
#                 if z2_actual > 0.1 and z2_exp > 0.1:
#                     diff = z2_exp - z2_actual
#                     if diff > self.obstacle_height_thresh or diff < -self.ditch_depth_thresh:
#                         r75_hazards[i] = True

#         # Clean noise
#         r65_hazards_cleaned = self.filter_isolated_noise(r65_hazards)
#         r75_hazards_cleaned = self.filter_isolated_noise(r75_hazards)

#         # Generate Scans
#         scan_65 = self.create_base_scan(depth_msg, width)
#         scan_75 = self.create_base_scan(depth_msg, width)

#         ranges_65 = [float('inf')] * width
#         ranges_75 = [float('inf')] * width

#         pixel_offsets = np.arange(width) - cx
#         cos_angles = np.cos(np.arctan(pixel_offsets / focal_length))

#         for i in range(width):
#             cos_a = cos_angles[i]

#             # Row 65% temporal history
#             if r65_hazards_cleaned[i]:
#                 self.r65_consecutive_counts[i] += 1
#             else:
#                 self.r65_consecutive_counts[i] = 0

#             if self.r65_consecutive_counts[i] >= self.history_depth:
#                 ranges_65[i] = float(row1_data[i]) / cos_a

#             # Row 75% temporal history
#             if r75_hazards_cleaned[i]:
#                 self.r75_consecutive_counts[i] += 1
#             else:
#                 self.r75_consecutive_counts[i] = 0

#             if self.r75_consecutive_counts[i] >= self.history_depth:
#                 ranges_75[i] = float(row2_data[i]) / cos_a

#         scan_65.ranges = ranges_65
#         scan_75.ranges = ranges_75

#         # Cache latest processed scans for timer loop
#         self.latest_scan_65 = scan_65
#         self.latest_scan_75 = scan_75

#     # --------------------------------------------------------------------------
#     # High-Frequency 30 Hz Timer Callback
#     # --------------------------------------------------------------------------
#     def timer_publish_callback(self):
#         now_stamp = self.get_clock().now().to_msg()

#         # Publish Diagnostic Slope Signals
#         self.pub_slope.publish(Bool(data=self.latest_slope_flag))
#         self.pub_mod_slope.publish(Bool(data=self.latest_mod_slope_flag))

#         # Publish Costmap LaserScan Messages with updated timestamps
#         if self.latest_scan_65 is not None and self.latest_scan_75 is not None:
#             self.latest_scan_65.header.stamp = now_stamp
#             self.latest_scan_75.header.stamp = now_stamp

#             self.pub_obs_65.publish(self.latest_scan_65)
#             self.pub_obs_75.publish(self.latest_scan_75)

#     def create_base_scan(self, msg, width):
#         scan = LaserScan()
#         scan.header.frame_id = msg.header.frame_id
#         scan.angle_min = -(self.hfov_rad / 2.0)
#         scan.angle_max = (self.hfov_rad / 2.0)
#         scan.angle_increment = (scan.angle_max - scan.angle_min) / (width - 1)
#         scan.range_min = 0.1
#         scan.range_max = 10.0
#         return scan


# def main(args=None):
#     rclpy.init(args=args)
#     node = HighFreqDepthObstacleDetector()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         if rclpy.ok():
#             rclpy.shutdown()


# if __name__ == '__main__':
#     main()



####################### Now detecting the obstacles but frequency is limited to 9-10Hz (slope video was recorded on this script)
import math
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import message_filters


class RGBDGuidedDepthDetector(Node):
    def __init__(self):
        super().__init__('rgbd_guided_depth_detector')

        self.bridge = CvBridge()

        # --- TOPICS CONFIGURATION ---
        self.input_depth_topic = '/camera/camera/aligned_depth_to_color/image_raw'
        self.input_rgb_topic = '/camera/camera/color/image_raw'

        self.output_obstacles_65_topic = '/detected_obstacles_65'
        self.output_obstacles_75_topic = '/detected_obstacles_75'

        self.output_slope_topic = '/obstacle/slope_detected'
        self.output_moderate_slope_topic = '/obstacle/moderate_slope_detected'

        # --- PARAMETERS ---
        self.declare_parameter('row1_pct', 0.65)
        self.declare_parameter('row2_pct', 0.75)
        self.declare_parameter('min_slope_threshold_deg', 12.0)
        self.declare_parameter('max_slope_threshold_deg', 17.0)

        self.row1_pct = self.get_parameter('row1_pct').get_parameter_value().double_value
        self.row2_pct = self.get_parameter('row2_pct').get_parameter_value().double_value
        self.min_slope_deg = self.get_parameter('min_slope_threshold_deg').get_parameter_value().double_value
        self.max_slope_deg = self.get_parameter('max_slope_threshold_deg').get_parameter_value().double_value

        # Motion & Sensor Config
        self.obstacle_height_thresh = 0.08  # 8 cm height off fitted plane = obstacle
        self.ditch_depth_thresh = 0.10     # 12 cm drop below fitted plane = ditch
        self.max_detection_range = 3.00    # 3.0 m max range
        self.hfov_rad = 1.50               # ~86 deg
        self.v_window = 5

        # Spatial & Temporal Filters (Tightened to prevent missing moving/small obstacles)
        self.min_cluster_width = 3
        self.history_depth = 2

        # Calibration State
        self.calibrated = False
        self.calibration_frames_needed = 20
        self.baseline_normal_samples = []
        self.baseline_floor_normal = None

        # Temporal History Buffers
        self.r65_consecutive_counts = None
        self.r75_consecutive_counts = None

        # --- SUBSCRIPTIONS ---
        self.depth_sub = message_filters.Subscriber(self, Image, self.input_depth_topic)
        self.rgb_sub = message_filters.Subscriber(self, Image, self.input_rgb_topic)

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.depth_sub, self.rgb_sub], queue_size=30, slop=0.15
        )
        self.sync.registerCallback(self.synchronized_callback)

        # --- PUBLISHERS ---
        self.pub_obs_65 = self.create_publisher(LaserScan, self.output_obstacles_65_topic, 10)
        self.pub_obs_75 = self.create_publisher(LaserScan, self.output_obstacles_75_topic, 10)
        self.pub_slope = self.create_publisher(Bool, self.output_slope_topic, 10)
        self.pub_mod_slope = self.create_publisher(Bool, self.output_moderate_slope_topic, 10)

        self.get_logger().info(
            f"Refactored RGB-D Ground Surface Monitor Initialized.\n"
            f"Slope Navigability Window: [{self.min_slope_deg}° - {self.max_slope_deg}°]"
        )

    # --------------------------------------------------------------------------
    # Helper: Extract Clean 1D Depth Profile
    # --------------------------------------------------------------------------
    def get_guided_row_profile(self, depth_matrix, center_row_idx, width):
        half_w = self.v_window // 2
        start_row = max(0, center_row_idx - half_w)
        end_row = min(depth_matrix.shape[0], center_row_idx + half_w + 1)

        depth_block = depth_matrix[start_row:end_row, :]
        valid_mask = (depth_block > 0.1) & (depth_block < self.max_detection_range)
        
        with np.errstate(all='ignore'):
            block_with_nans = np.where(valid_mask, depth_block, np.nan)
            all_nan_cols = np.all(np.isnan(block_with_nans), axis=0)
            composite_row = np.nanmedian(block_with_nans, axis=0)
            composite_row[all_nan_cols] = 0.0

        return np.nan_to_num(composite_row, nan=0.0)

    # --------------------------------------------------------------------------
    # Helper: Spatial Cluster Filter
    # --------------------------------------------------------------------------
    def filter_isolated_noise(self, hazard_mask):
        kernel = np.ones((1, self.min_cluster_width), dtype=np.uint8)
        cleaned_mask = cv2.morphologyEx(hazard_mask.astype(np.uint8), cv2.MORPH_OPEN, kernel)
        return cleaned_mask.astype(bool)

    # --------------------------------------------------------------------------
    # Helper: Vectorized 3D RANSAC Plane Fitting + Returns Plane Equation
    # --------------------------------------------------------------------------
    def fit_ransac_plane(self, depth_meters, row_start_pct=0.60, row_end_pct=0.90, sample_stride=4):
        height, width = depth_meters.shape
        start_row = int(height * row_start_pct)
        end_row = int(height * row_end_pct)

        v_idx, u_idx = np.mgrid[start_row:end_row:sample_stride, 0:width:sample_stride]
        z = depth_meters[start_row:end_row:sample_stride, 0:width:sample_stride]

        valid_mask = (z > 0.3) & (z < self.max_detection_range)
        num_valid = np.sum(valid_mask)
        if num_valid < 50:
            return None, 0, None

        cx = width / 2.0
        cy = height / 2.0
        focal_length = cx / math.tan(self.hfov_rad / 2.0)

        z_valid = z[valid_mask]
        u_valid = u_idx[valid_mask]
        v_valid = v_idx[valid_mask]

        x_valid = (u_valid - cx) * z_valid / focal_length
        y_valid = (v_valid - cy) * z_valid / focal_length

        points = np.column_stack((x_valid, y_valid, z_valid))

        max_iterations = 40
        distance_threshold = 0.03

        idx = np.random.randint(0, num_valid, size=(max_iterations, 3))
        p1 = points[idx[:, 0]]
        p2 = points[idx[:, 1]]
        p3 = points[idx[:, 2]]

        v1 = p2 - p1
        v2 = p3 - p1
        normals = np.cross(v1, v2)
        norms = np.linalg.norm(normals, axis=1, keepdims=True)

        valid_normals = (norms.squeeze() > 1e-6)
        if not np.any(valid_normals):
            return None, 0, None

        normals = np.divide(normals, norms, out=np.zeros_like(normals), where=norms > 1e-6)

        d = -np.einsum('ij,ij->i', normals, p1)
        dist_matrix = np.abs(np.dot(normals, points.T) + d[:, np.newaxis])

        inliers_counts = np.sum(dist_matrix < distance_threshold, axis=1)
        best_idx = np.argmax(inliers_counts)

        best_inliers_count = int(inliers_counts[best_idx])
        best_normal = normals[best_idx]
        best_d = d[best_idx]

        if best_normal is not None and best_normal[1] > 0:
            best_normal = -best_normal
            best_d = -best_d

        plane_eq = (best_normal[0], best_normal[1], best_normal[2], best_d)
        return best_normal, best_inliers_count, plane_eq

    # --------------------------------------------------------------------------
    # Main Callback
    # --------------------------------------------------------------------------
    def synchronized_callback(self, depth_msg: Image, rgb_msg: Image):
        try:
            cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Failed to parse depth frame: {str(e)}")
            return

        if cv_depth.dtype == np.uint16:
            depth_meter = cv_depth.astype(np.float32) / 1000.0
        else:
            depth_meter = cv_depth.astype(np.float32)

        height, width = depth_meter.shape

        if self.r65_consecutive_counts is None:
            self.r65_consecutive_counts = np.zeros(width, dtype=np.int32)
            self.r75_consecutive_counts = np.zeros(width, dtype=np.int32)

        row1_idx = int(height * self.row1_pct)
        row2_idx = int(height * self.row2_pct)

        row1_data = self.get_guided_row_profile(depth_meter, row1_idx, width)
        row2_data = self.get_guided_row_profile(depth_meter, row2_idx, width)

        current_normal, inlier_count, plane_eq = self.fit_ransac_plane(depth_meter)

        # --- CALIBRATION PHASE ---
        if not self.calibrated:
            if current_normal is not None and inlier_count > 200:
                self.baseline_normal_samples.append(current_normal)

            if len(self.baseline_normal_samples) >= self.calibration_frames_needed:
                avg_normal = np.median(self.baseline_normal_samples, axis=0)
                self.baseline_floor_normal = avg_normal / np.linalg.norm(avg_normal)
                self.calibrated = True
                self.get_logger().info("--> CALIBRATION COMPLETE! Ground plane baseline registered.")
            return

        # --- SLOPE EVALUATION ---
        slope_detected_flag = False
        moderate_slope_flag = False

        if current_normal is not None and inlier_count > 150:
            dot_product = np.clip(np.dot(current_normal, self.baseline_floor_normal), -1.0, 1.0)
            angle_diff_deg = math.degrees(math.acos(dot_product))

            if self.min_slope_deg <= angle_diff_deg <= self.max_slope_deg:
                slope_detected_flag = True

            if self.min_slope_deg <= angle_diff_deg <= 15.0:
                moderate_slope_flag = True

        self.pub_slope.publish(Bool(data=slope_detected_flag))
        self.pub_mod_slope.publish(Bool(data=moderate_slope_flag))

        # --- PLANE-RELATIVE HAZARD DETECTION ---
        r65_hazards = np.zeros(width, dtype=bool)
        r75_hazards = np.zeros(width, dtype=bool)

        cx = width / 2.0
        cy = height / 2.0
        focal_length = cx / math.tan(self.hfov_rad / 2.0)

        if plane_eq is not None:
            A, B, C, D = plane_eq

            # Compute expected floor depth for Row 1 & Row 2
            u_indices = np.arange(width)
            x_dir = (u_indices - cx) / focal_length
            
            # Row 65% expected ground depth
            y_dir_1 = (row1_idx - cy) / focal_length
            denom_1 = (A * x_dir + B * y_dir_1 + C)
            expected_z1 = np.where(np.abs(denom_1) > 1e-5, -D / denom_1, 0.0)

            # Row 75% expected ground depth
            y_dir_2 = (row2_idx - cy) / focal_length
            denom_2 = (A * x_dir + B * y_dir_2 + C)
            expected_z2 = np.where(np.abs(denom_2) > 1e-5, -D / denom_2, 0.0)

            for i in range(width):
                # Row 65% Evaluation
                z1_actual = row1_data[i]
                z1_exp = expected_z1[i]
                if z1_actual > 0.1 and z1_exp > 0.1:
                    diff = z1_exp - z1_actual  # Positive = obstacle (closer than plane), Negative = ditch
                    if diff > self.obstacle_height_thresh or diff < -self.ditch_depth_thresh:
                        r65_hazards[i] = True

                # Row 75% Evaluation
                z2_actual = row2_data[i]
                z2_exp = expected_z2[i]
                if z2_actual > 0.1 and z2_exp > 0.1:
                    diff = z2_exp - z2_actual
                    if diff > self.obstacle_height_thresh or diff < -self.ditch_depth_thresh:
                        r75_hazards[i] = True

        # Apply Spatial Noise Filter
        r65_hazards_cleaned = self.filter_isolated_noise(r65_hazards)
        r75_hazards_cleaned = self.filter_isolated_noise(r75_hazards)

        # --- COSTMAP SCANS GENERATION ---
        scan_65 = self.create_base_scan(depth_msg, width)
        scan_75 = self.create_base_scan(depth_msg, width)

        ranges_65 = [float('inf')] * width
        ranges_75 = [float('inf')] * width

        pixel_offsets = np.arange(width) - cx
        cos_angles = np.cos(np.arctan(pixel_offsets / focal_length))

        for i in range(width):
            cos_a = cos_angles[i]

            # Row 65% temporal check
            if r65_hazards_cleaned[i]:
                self.r65_consecutive_counts[i] += 1
            else:
                self.r65_consecutive_counts[i] = 0

            if self.r65_consecutive_counts[i] >= self.history_depth:
                ranges_65[i] = float(row1_data[i]) / cos_a

            # Row 75% temporal check
            if r75_hazards_cleaned[i]:
                self.r75_consecutive_counts[i] += 1
            else:
                self.r75_consecutive_counts[i] = 0

            if self.r75_consecutive_counts[i] >= self.history_depth:
                ranges_75[i] = float(row2_data[i]) / cos_a

        scan_65.ranges = ranges_65
        scan_75.ranges = ranges_75

        self.pub_obs_65.publish(scan_65)
        self.pub_obs_75.publish(scan_75)

    def create_base_scan(self, msg, width):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = msg.header.frame_id
        scan.angle_min = -(self.hfov_rad / 2.0)
        scan.angle_max = (self.hfov_rad / 2.0)
        scan.angle_increment = (scan.angle_max - scan.angle_min) / (width - 1)
        scan.range_min = 0.1
        scan.range_max = 10.0
        return scan


def main(args=None):
    rclpy.init(args=args)
    node = RGBDGuidedDepthDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()



############# SLOPE IS DETECTING PROPERLY But missing the obstacles sometime. Frequency is about 12 - 16 Hz

# import math
# import numpy as np
# import cv2

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image, LaserScan
# from std_msgs.msg import Bool
# from cv_bridge import CvBridge
# import message_filters


# class RGBDGuidedDepthDetector(Node):
#     def __init__(self):
#         super().__init__('rgbd_guided_depth_detector')

#         self.bridge = CvBridge()

#         # --- TOPICS CONFIGURATION ---
#         self.input_depth_topic = '/camera/camera/aligned_depth_to_color/image_raw'
#         self.input_rgb_topic = '/camera/camera/color/image_raw'

#         # Costmap Topics (Both Rows Published Separately)
#         self.output_obstacles_65_topic = '/detected_obstacles_65'
#         self.output_obstacles_75_topic = '/detected_obstacles_75'

#         # Diagnostic Slope Topics
#         self.output_slope_topic = '/obstacle/slope_detected'
#         self.output_moderate_slope_topic = '/obstacle/moderate_slope_detected'

#         # --- PARAMETERS ---
#         self.declare_parameter('row1_pct', 0.65)
#         self.declare_parameter('row2_pct', 0.75)
#         self.declare_parameter('slope_threshold_deg', 6.0)

#         self.row1_pct = self.get_parameter('row1_pct').get_parameter_value().double_value
#         self.row2_pct = self.get_parameter('row2_pct').get_parameter_value().double_value
#         self.slope_threshold_deg = self.get_parameter('slope_threshold_deg').get_parameter_value().double_value

#         # Motion & Sensor Config
#         self.ditch_threshold = 0.12        # 12 cm
#         self.obstacle_threshold = 0.10     # 10 cm
#         self.max_detection_range = 2.00    # 2.0 m
#         self.hfov_rad = 1.50               # ~86 deg
#         self.v_window = 9

#         # Bilateral Filtering Params
#         self.bilateral_diameter = 5
#         self.sigma_color = 40.0
#         self.sigma_space = 5.0

#         # Spatial & Temporal Filters
#         self.min_cluster_width = 7
#         self.history_depth = 7

#         # Calibration
#         self.calibrated = False
#         self.calibration_frames_needed = 30
#         self.row1_samples = []
#         self.row2_samples = []
#         self.baseline_normal_samples = []

#         self.row1_baseline = 0.0
#         self.row2_baseline = 0.0
#         self.baseline_floor_normal = None

#         # Temporal History Buffers (Tracked independently for each row)
#         self.r65_consecutive_counts = None
#         self.r75_consecutive_counts = None

#         # --- SUBSCRIPTIONS ---
#         self.depth_sub = message_filters.Subscriber(self, Image, self.input_depth_topic)
#         self.rgb_sub = message_filters.Subscriber(self, Image, self.input_rgb_topic)

#         self.sync = message_filters.ApproximateTimeSynchronizer(
#             [self.depth_sub, self.rgb_sub], queue_size=30, slop=0.15
#         )
#         self.sync.registerCallback(self.synchronized_callback)

#         # --- PUBLISHERS ---
#         self.pub_obs_65 = self.create_publisher(LaserScan, self.output_obstacles_65_topic, 10)
#         self.pub_obs_75 = self.create_publisher(LaserScan, self.output_obstacles_75_topic, 10)
#         self.pub_slope = self.create_publisher(Bool, self.output_slope_topic, 10)
#         self.pub_mod_slope = self.create_publisher(Bool, self.output_moderate_slope_topic, 10)

#         self.get_logger().info("RGBD Dual-Row Obstacle & RANSAC Slope Detector Node Launched.")

#     # --------------------------------------------------------------------------
#     # Helper: Extract Clean 1D Depth Profile
#     # --------------------------------------------------------------------------
#     def get_guided_row_profile(self, depth_matrix, rgb_matrix, center_row_idx, width):
#         half_w = self.v_window // 2
#         start_row = max(0, center_row_idx - half_w)
#         end_row = min(depth_matrix.shape[0], center_row_idx + half_w + 1)

#         depth_block = depth_matrix[start_row:end_row, :]
#         depth_scaled = np.clip(depth_block * 100.0, 0, 255).astype(np.uint8)

#         filtered_depth_block = cv2.bilateralFilter(
#             src=depth_scaled,
#             d=self.bilateral_diameter,
#             sigmaColor=self.sigma_color,
#             sigmaSpace=self.sigma_space
#         )

#         depth_block_meters = filtered_depth_block.astype(np.float32) / 100.0

#         valid_mask = (depth_block_meters > 0.1)
#         with np.errstate(all='ignore'):
#             block_with_nans = np.where(valid_mask, depth_block_meters, np.nan)
#             all_nan_cols = np.all(np.isnan(block_with_nans), axis=0)
#             composite_row = np.nanmedian(block_with_nans, axis=0)
#             composite_row[all_nan_cols] = 0.0

#         composite_row = np.nan_to_num(composite_row, nan=0.0)

#         # Scaled median blur conversion (safe for OpenCV uint8 limitation)
#         composite_scaled = np.clip(composite_row * 100.0, 0, 255).astype(np.uint8)
#         composite_scaled_matrix = composite_scaled.reshape(1, width)
#         filtered_scaled = cv2.medianBlur(composite_scaled_matrix, 5).flatten()

#         return filtered_scaled.astype(np.float32) / 100.0

#     # --------------------------------------------------------------------------
#     # Helper: Spatial Cluster Filter
#     # --------------------------------------------------------------------------
#     def filter_isolated_noise(self, hazard_mask):
#         kernel = np.ones((1, self.min_cluster_width), dtype=np.uint8)
#         cleaned_mask = cv2.morphologyEx(hazard_mask.astype(np.uint8), cv2.MORPH_OPEN, kernel)
#         return cleaned_mask.astype(bool)

#     # --------------------------------------------------------------------------
#     # Helper: Fast Fully Vectorized 3D RANSAC Plane Fitting
#     # --------------------------------------------------------------------------
#     def fit_ransac_plane(self, depth_meters, row_start_pct=0.65, row_end_pct=0.80, sample_stride=7):
#         height, width = depth_meters.shape
#         start_row = int(height * row_start_pct)
#         end_row = int(height * row_end_pct)

#         v_idx, u_idx = np.mgrid[start_row:end_row:sample_stride, 0:width:sample_stride]
#         z = depth_meters[start_row:end_row:sample_stride, 0:width:sample_stride]

#         valid_mask = (z > 0.3) & (z < self.max_detection_range)
#         num_valid = np.sum(valid_mask)
#         if num_valid < 30:
#             return None, 0

#         cx = width / 2.0
#         cy = height / 2.0
#         focal_length = cx / math.tan(self.hfov_rad / 2.0)

#         z_valid = z[valid_mask]
#         u_valid = u_idx[valid_mask]
#         v_valid = v_idx[valid_mask]

#         x_valid = (u_valid - cx) * z_valid / focal_length
#         y_valid = (v_valid - cy) * z_valid / focal_length

#         points = np.column_stack((x_valid, y_valid, z_valid))

#         max_iterations = 30
#         distance_threshold = 0.03  # 3cm plane tolerance

#         idx = np.random.randint(0, num_valid, size=(max_iterations, 3))
#         p1 = points[idx[:, 0]]
#         p2 = points[idx[:, 1]]
#         p3 = points[idx[:, 2]]

#         v1 = p2 - p1
#         v2 = p3 - p1
#         normals = np.cross(v1, v2)
#         norms = np.linalg.norm(normals, axis=1, keepdims=True)

#         valid_normals = (norms.squeeze() > 1e-6)
#         if not np.any(valid_normals):
#             return None, 0

#         normals = np.divide(normals, norms, out=np.zeros_like(normals), where=norms > 1e-6)

#         d = -np.einsum('ij,ij->i', normals, p1)
#         dist_matrix = np.abs(np.dot(normals, points.T) + d[:, np.newaxis])

#         inliers_counts = np.sum(dist_matrix < distance_threshold, axis=1)
#         best_idx = np.argmax(inliers_counts)

#         best_inliers_count = int(inliers_counts[best_idx])
#         best_normal = normals[best_idx]

#         if best_normal is not None and best_normal[1] > 0:
#             best_normal = -best_normal

#         return best_normal, best_inliers_count

#     # --------------------------------------------------------------------------
#     # Main Synchronized Callback
#     # --------------------------------------------------------------------------
#     def synchronized_callback(self, depth_msg: Image, rgb_msg: Image):
#         try:
#             cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
#             cv_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
#         except Exception as e:
#             self.get_logger().error(f"Failed to parse frames: {str(e)}")
#             return

#         if cv_depth.dtype == np.uint16:
#             depth_meter = cv_depth.astype(np.float32) / 1000.0
#         else:
#             depth_meter = cv_depth.astype(np.float32)

#         height, width = depth_meter.shape

#         if self.r65_consecutive_counts is None:
#             self.r65_consecutive_counts = np.zeros(width, dtype=np.int32)
#             self.r75_consecutive_counts = np.zeros(width, dtype=np.int32)

#         row1_idx = int(height * self.row1_pct)
#         row2_idx = int(height * self.row2_pct)

#         row1_data = self.get_guided_row_profile(depth_meter, cv_rgb, row1_idx, width)
#         row2_data = self.get_guided_row_profile(depth_meter, cv_rgb, row2_idx, width)

#         current_normal, inlier_count = self.fit_ransac_plane(depth_meter)

#         # --- STARTUP CALIBRATION PHASE ---
#         if not self.calibrated:
#             quarter_w = width // 4
#             r1_seg = row1_data[quarter_w : 3 * quarter_w]
#             r2_seg = row2_data[quarter_w : 3 * quarter_w]
#             v_r1 = r1_seg[r1_seg > 0.1]
#             v_r2 = r2_seg[r2_seg > 0.1]

#             if len(v_r1) > 5 and len(v_r2) > 5 and current_normal is not None and inlier_count > 200:
#                 self.row1_samples.append(np.median(v_r1))
#                 self.row2_samples.append(np.median(v_r2))
#                 self.baseline_normal_samples.append(current_normal)
#             else:
#                 return

#             if len(self.row1_samples) >= self.calibration_frames_needed:
#                 self.row1_baseline = float(np.median(self.row1_samples))
#                 self.row2_baseline = float(np.median(self.row2_samples))

#                 avg_normal = np.median(self.baseline_normal_samples, axis=0)
#                 self.baseline_floor_normal = avg_normal / np.linalg.norm(avg_normal)
#                 self.calibrated = True

#                 self.get_logger().info(
#                     f"--> CALIBRATION COMPLETE!\n"
#                     f"    Row 65% Base: {self.row1_baseline:.2f}m | Row 75% Base: {self.row2_baseline:.2f}m\n"
#                     f"    Baseline Normal: [{self.baseline_floor_normal[0]:.3f}, {self.baseline_floor_normal[1]:.3f}, {self.baseline_floor_normal[2]:.3f}]"
#                 )
#             else:
#                 return

#         # --- SLOPE EVALUATION (MATH PLANE) ---
#         slope_detected_flag = False
#         moderate_slope_flag = False

#         if current_normal is not None and inlier_count > 200:
#             dot_product = np.clip(np.dot(current_normal, self.baseline_floor_normal), -1.0, 1.0)
#             angle_diff_deg = math.degrees(math.acos(dot_product))

#             if angle_diff_deg >= self.slope_threshold_deg:
#                 slope_detected_flag = True

#             if 5.0 <= angle_diff_deg <= 15.0:
#                 moderate_slope_flag = True

#         # Publish Diagnostic Slope Topics
#         slope_msg = Bool()
#         slope_msg.data = slope_detected_flag
#         self.pub_slope.publish(slope_msg)

#         mod_slope_msg = Bool()
#         mod_slope_msg.data = moderate_slope_flag
#         self.pub_mod_slope.publish(mod_slope_msg)

#         # --- INDEPENDENT ROW HAZARD DETECTION ---
#         r65_hazards = np.zeros(width, dtype=bool)
#         r75_hazards = np.zeros(width, dtype=bool)

#         # Only check hazards if NO slope is detected mathematically
#         if not slope_detected_flag:
#             for i in range(width):
#                 # Row 65% evaluation
#                 r1_dist = float(row1_data[i])
#                 if r1_dist > 0.0:
#                     if (r1_dist > (self.row1_baseline + self.ditch_threshold) and r1_dist <= self.max_detection_range) or \
#                        (r1_dist < (self.row1_baseline - self.obstacle_threshold)):
#                         r65_hazards[i] = True

#                 # Row 75% evaluation
#                 r2_dist = float(row2_data[i])
#                 if r2_dist > 0.0:
#                     if (r2_dist > (self.row2_baseline + self.ditch_threshold) and r2_dist <= self.max_detection_range) or \
#                        (r2_dist < (self.row2_baseline - self.obstacle_threshold)):
#                         r75_hazards[i] = True

#         # Apply Spatial Noise Filter
#         r65_hazards_cleaned = self.filter_isolated_noise(r65_hazards)
#         r75_hazards_cleaned = self.filter_isolated_noise(r75_hazards)

#         # --- TEMPORAL INTEGRATION & COSTMAP SCANS GENERATION ---
#         scan_65 = self.create_base_scan(depth_msg, width)
#         scan_75 = self.create_base_scan(depth_msg, width)

#         ranges_65 = [float('inf')] * width
#         ranges_75 = [float('inf')] * width

#         cx = width / 2.0
#         focal_length = cx / math.tan(self.hfov_rad / 2.0)
#         pixel_offsets = np.arange(width) - cx
#         cos_angles = np.cos(np.arctan(pixel_offsets / focal_length))

#         for i in range(width):
#             cos_a = cos_angles[i]

#             # Process Row 65%
#             if r65_hazards_cleaned[i]:
#                 self.r65_consecutive_counts[i] += 1
#             else:
#                 self.r65_consecutive_counts[i] = 0

#             if self.r65_consecutive_counts[i] >= self.history_depth:
#                 ranges_65[i] = float(row1_data[i]) / cos_a

#             # Process Row 75%
#             if r75_hazards_cleaned[i]:
#                 self.r75_consecutive_counts[i] += 1
#             else:
#                 self.r75_consecutive_counts[i] = 0

#             if self.r75_consecutive_counts[i] >= self.history_depth:
#                 ranges_75[i] = float(row2_data[i]) / cos_a

#         scan_65.ranges = ranges_65
#         scan_75.ranges = ranges_75

#         # Publish both row topics for costmap
#         self.pub_obs_65.publish(scan_65)
#         self.pub_obs_75.publish(scan_75)

#     def create_base_scan(self, msg, width):
#         scan = LaserScan()
#         scan.header.stamp = self.get_clock().now().to_msg()
#         scan.header.frame_id = msg.header.frame_id
#         scan.angle_min = -(self.hfov_rad / 2.0)
#         scan.angle_max = (self.hfov_rad / 2.0)
#         scan.angle_increment = (scan.angle_max - scan.angle_min) / (width - 1)
#         scan.range_min = 0.1
#         scan.range_max = 10.0
#         return scan


# def main(args=None):
#     rclpy.init(args=args)
#     node = RGBDGuidedDepthDetector()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         if rclpy.ok():
#             rclpy.shutdown()


# if __name__ == '__main__':
#     main()


####################################################Integrate with RGB 
# import rclpy
# from rclpy.node import Node
# import numpy as np
# import cv2
# import math
# from cv_bridge import CvBridge

# # ROS 2 message types
# from sensor_msgs.msg import Image, LaserScan
# from rclpy.qos import qos_profile_sensor_data

# # ROS 2 Message Filters for Time Synchronization
# import message_filters

# class RGBDGuidedDepthDetector(Node):
#     def __init__(self):
#         super().__init__('rgbd_guided_depth_detector')
        
#         self.bridge = CvBridge()
        
#         # --- TOPICS CONFIGURATION ---
#         self.input_depth_topic = '/camera/camera/aligned_depth_to_color/image_raw'
#         self.input_rgb_topic = '/camera/camera/color/image_raw'
        
#         self.output_scan_65_topic = '/detected_obstacles_65'
#         self.output_scan_75_topic = '/detected_obstacles_75'

#         # This registers the parameters with ROS 2 and sets default fallback values
#         self.declare_parameter('row1_pct', 0.65)
#         self.declare_parameter('row2_pct', 0.75)

#         # This reads the values dynamically from a launch file, YAML, or command line
#         self.row1_pct = self.get_parameter('row1_pct').get_parameter_value().double_value
#         self.row2_pct = self.get_parameter('row2_pct').get_parameter_value().double_value
        
#         # Motion Robust Thresholds
#         self.ditch_threshold = 0.06     
#         self.obstacle_threshold = 0.10  
#         self.max_detection_range = 2.00 
#         self.hfov_rad = 1.50            
        
#         # Target window Configuration  
#         self.v_window = 9               
        
#         # Bilateral Tuning
#         self.bilateral_diameter = 5      
#         self.sigma_color = 40.0     #25.0          
#         self.sigma_space = 5.0           
        
#         # --- NEW: SPATIAL ADJACENCY FILTER CONFIG ---
#         self.min_cluster_width = 7      # Hazard must be at least 5 pixels wide horizontally  (inital value 3)
        
#         # Startup Calibration
#         self.calibrated = False
#         self.calibration_frames_needed = 30  
#         self.row1_samples = []
#         self.row2_samples = []
#         self.row1_baseline = 0.0
#         self.row2_baseline = 0.0
        
#         # Temporal History Buffers 
#         self.history_depth = 7  
#         self.r65_consecutive_counts = None
#         self.r75_consecutive_counts = None
        
#         # Diagnostic tracking metrics
#         self.depth_received_count = 0
#         self.rgb_received_count = 0

#         # --- SUBSCRIPTIONS ---
#         self.depth_test_sub = self.create_subscription(Image, self.input_depth_topic, self.depth_diagnostic_cb, 10)
#         self.rgb_test_sub = self.create_subscription(Image, self.input_rgb_topic, self.rgb_diagnostic_cb, 10)

#         self.depth_sub = message_filters.Subscriber(self, Image, self.input_depth_topic, qos_profile=10)
#         self.rgb_sub = message_filters.Subscriber(self, Image, self.input_rgb_topic, qos_profile=10)
        
#         self.sync = message_filters.ApproximateTimeSynchronizer([self.depth_sub, self.rgb_sub], queue_size=30, slop=0.15)
#         self.sync.registerCallback(self.synchronized_callback)
            
#         self.pub_65 = self.create_publisher(LaserScan, self.output_scan_65_topic, 10)
#         self.pub_75 = self.create_publisher(LaserScan, self.output_scan_75_topic, 10)
            
#         self.get_logger().info("Spatial-Temporal Density Denoise Node Launched.")

#     def depth_diagnostic_cb(self, msg):
#         self.depth_received_count += 1

#     def rgb_diagnostic_cb(self, msg):
#         self.rgb_received_count += 1

#     def get_guided_row_profile(self, depth_matrix, rgb_matrix, center_row_idx, width):
#         half_w = self.v_window // 2
#         start_row = max(0, center_row_idx - half_w)
#         end_row = min(depth_matrix.shape[0], center_row_idx + half_w + 1)
        
#         depth_block = depth_matrix[start_row:end_row, :]
#         depth_scaled = np.clip(depth_block * 100.0, 0, 255).astype(np.uint8)
        
#         filtered_depth_block = cv2.bilateralFilter(
#             src=depth_scaled,
#             d=self.bilateral_diameter,
#             sigmaColor=self.sigma_color,
#             sigmaSpace=self.sigma_space
#         )
        
#         depth_block_meters = filtered_depth_block.astype(np.float32) / 100.0
        
#         valid_mask = (depth_block_meters > 0.1)
#         with np.errstate(all='ignore'):
#             block_with_nans = np.where(valid_mask, depth_block_meters, np.nan)
#             all_nan_cols = np.all(np.isnan(block_with_nans), axis=0)
#             composite_row = np.nanmedian(block_with_nans, axis=0)
#             composite_row[all_nan_cols] = 0.0
            
#         composite_row = np.nan_to_num(composite_row, nan=0.0)
        
#         composite_row_matrix = composite_row.reshape(1, width)
#         final_clean_row = cv2.medianBlur(composite_row_matrix, 5).flatten()
#         return final_clean_row

#     def filter_isolated_noise(self, hazard_mask):
#         """
#         Uses a morphological opening operation to instantly wipe away 
#         isolated pixel groups that don't satisfy the minimum cluster width.
#         """
#         kernel = np.ones((1, self.min_cluster_width), dtype=np.uint8)
#         cleaned_mask = cv2.morphologyEx(hazard_mask.astype(np.uint8), cv2.MORPH_OPEN, kernel)
#         return cleaned_mask.astype(bool)

#     def synchronized_callback(self, depth_msg: Image, rgb_msg: Image):
#         try:
#             cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
#             cv_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
#         except Exception as e:
#             self.get_logger().error(f"Failed to parse frames: {str(e)}")
#             return

#         depth_meter = cv_depth.astype(np.float32) / 1000.0
#         height, width = depth_meter.shape
        
#         if self.r65_consecutive_counts is None:
#             self.r65_consecutive_counts = np.zeros(width, dtype=np.int32)
#             self.r75_consecutive_counts = np.zeros(width, dtype=np.int32)

#         row1_idx = int(height * self.row1_pct)
#         row2_idx = int(height * self.row2_pct)
        
#         row1_data = self.get_guided_row_profile(depth_meter, cv_rgb, row1_idx, width)
#         row2_data = self.get_guided_row_profile(depth_meter, cv_rgb, row2_idx, width)

#         # --- STARTUP CALIBRATION PHASE ---
#         if not self.calibrated:
#             quarter_w = width // 4
#             r1_seg = row1_data[quarter_w : 3 * quarter_w]
#             r2_seg = row2_data[quarter_w : 3 * quarter_w]
#             v_r1 = r1_seg[r1_seg > 0.1]
#             v_r2 = r2_seg[r2_seg > 0.1]
            
#             if len(v_r1) > 5 and len(v_r2) > 5:
#                 self.row1_samples.append(np.median(v_r1))
#                 self.row2_samples.append(np.median(v_r2))
#             else:
#                 return
                
#             if len(self.row1_samples) >= self.calibration_frames_needed:
#                 self.row1_baseline = float(np.median(self.row1_samples))
#                 self.row2_baseline = float(np.median(self.row2_samples))
#                 self.calibrated = True
#                 self.get_logger().info(f"--> CALIBRATION COMPLETE! Locked: 65%={self.row1_baseline:.2f}m, 75%={self.row2_baseline:.2f}m")
#             else:
#                 return

#         # --- ARRANGE BASE SCANS ---
#         scan_65 = self.create_base_scan(depth_msg, width)
#         scan_75 = self.create_base_scan(depth_msg, width)

#         ranges_65 = [float('inf')] * width
#         ranges_75 = [float('inf')] * width
        
#         cx = width / 2.0
#         focal_length = cx / math.tan(self.hfov_rad / 2.0)
#         pixel_offsets = np.arange(width) - cx
#         cos_angles = np.cos(np.arctan(pixel_offsets / focal_length))

#         # --- STEP 1: INITIAL PASS DETECTION MASKS ---
#         r65_hazards = np.zeros(width, dtype=bool)
#         r75_hazards = np.zeros(width, dtype=bool)

#         for i in range(width):
#             r1_dist = float(row1_data[i])
#             if r1_dist > 0.0:
#                 if (r1_dist > (self.row1_baseline + self.ditch_threshold) and r1_dist <= self.max_detection_range) or \
#                    (r1_dist < (self.row1_baseline - self.obstacle_threshold)):
#                     r65_hazards[i] = True

#             r2_dist = float(row2_data[i])
#             if r2_dist > 0.0:
#                 if (r2_dist > (self.row2_baseline + self.ditch_threshold) and r2_dist <= self.max_detection_range) or \
#                    (r2_dist < (self.row2_baseline - self.obstacle_threshold)):
#                     r75_hazards[i] = True

#         # --- STEP 2: APPLY SPATIAL ADJACENCY FILTER LAYER ---
#         # Wipes out isolated column noise spikes before updating temporal trackers
#         r65_hazards_cleaned = self.filter_isolated_noise(r65_hazards)
#         r75_hazards_cleaned = self.filter_isolated_noise(r75_hazards)

#         # --- STEP 3: TEMPORAL INTEGRATION PASS ---
#         for i in range(width):
#             cos_a = cos_angles[i]

#             # Process 65% Row
#             if r65_hazards_cleaned[i]:
#                 self.r65_consecutive_counts[i] += 1
#             else:
#                 self.r65_consecutive_counts[i] = 0
                
#             if self.r65_consecutive_counts[i] >= self.history_depth:
#                 ranges_65[i] = float(row1_data[i]) / cos_a

#             # Process 75% Row
#             if r75_hazards_cleaned[i]:
#                 self.r75_consecutive_counts[i] += 1
#             else:
#                 self.r75_consecutive_counts[i] = 0
                
#             if self.r75_consecutive_counts[i] >= self.history_depth:
#                 ranges_75[i] = float(row2_data[i]) / cos_a

#         scan_65.ranges = ranges_65
#         scan_75.ranges = ranges_75
        
#         self.pub_65.publish(scan_65)
#         self.pub_75.publish(scan_75)

#     def create_base_scan(self, msg, width):
#         scan = LaserScan()
#         scan.header.stamp = self.get_clock().now().to_msg()
#         scan.header.frame_id = msg.header.frame_id
#         scan.angle_min = -(self.hfov_rad / 2.0)
#         scan.angle_max = (self.hfov_rad / 2.0)
#         scan.angle_increment = (scan.angle_max - scan.angle_min) / (width - 1)
#         scan.range_min = 0.1
#         scan.range_max = 10.0
#         return scan

# def main(args=None):
#     rclpy.init(args=args)
#     node = RGBDGuidedDepthDetector()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()


############################################################
# import rclpy
# from rclpy.node import Node
# import numpy as np
# import cv2
# import math
# from cv_bridge import CvBridge

# from sensor_msgs.msg import Image, LaserScan
# from rclpy.qos import qos_profile_sensor_data
# import message_filters

# class RGBDGuidedDepthDetector(Node):
#     def __init__(self):
#         super().__init__('rgbd_guided_depth_detector')
        
#         self.bridge = CvBridge()
        
#         # --- PARAMETERS ---
#         # FIX: Ensure you are using the 'aligned_depth_to_color' topic!
#         self.input_depth_topic = '/camera/camera/aligned_depth_to_color/image_raw'
#         self.input_rgb_topic = '/camera/camera/color/image_raw'
        
#         self.output_scan_70_topic = '/detected_ditches_65'
#         self.output_scan_80_topic = '/detected_ditches_75'
        
#         self.ditch_threshold = 0.1     
#         self.obstacle_threshold = 0.1  
#         self.max_detection_range = 2.00 
#         self.hfov_rad = 1.50  
        
#         self.row1_pct = 0.65  
#         self.row2_pct = 0.75  
#         self.v_window = 9 
        
#         self.bilateral_diameter = 5      
#         self.sigma_color = 25.0          
#         self.sigma_space = 5.0           
        
#         self.calibrated = False
#         self.calibration_frames_needed = 30  
#         self.row1_samples = []
#         self.row2_samples = []
#         self.row1_baseline = 0.0
#         self.row2_baseline = 0.0
        
#         self.history_depth = 5  
#         self.r70_consecutive_counts = None
#         self.r80_consecutive_counts = None
        
#         # Diagnostic trackers to print in terminal
#         self.depth_received_count = 0
#         self.rgb_received_count = 0

#         # --- SUBSCRIPTIONS ---
#         # Standard subscriptions to verify basic traffic independent of the synchronizer
#         self.depth_test_sub = self.create_subscription(
#             Image, self.input_depth_topic, self.depth_diagnostic_cb, 10)
#         self.rgb_test_sub = self.create_subscription(
#             Image, self.input_rgb_topic, self.rgb_diagnostic_cb, 10)

#         # Message Filter wrappers targeting the exact same streams
#         self.depth_sub = message_filters.Subscriber(
#             self, Image, self.input_depth_topic, qos_profile=10)
#         self.rgb_sub = message_filters.Subscriber(
#             self, Image, self.input_rgb_topic, qos_profile=10)
        
#         # FIX: Increased slop from 0.01 to 0.10 (100ms) to accommodate processing jitter
#         self.sync = message_filters.ApproximateTimeSynchronizer(
#             [self.depth_sub, self.rgb_sub], queue_size=30, slop=0.15)
#         self.sync.registerCallback(self.synchronized_callback)
            
#         self.pub_70 = self.create_publisher(LaserScan, self.output_scan_70_topic, 10)
#         self.pub_80 = self.create_publisher(LaserScan, self.output_scan_80_topic, 10)
            
#         self.get_logger().info("Diagnostic Sync Node Launched.")

#     def depth_diagnostic_cb(self, msg):
#         self.depth_received_count += 1

#     def rgb_diagnostic_cb(self, msg):
#         self.rgb_received_count += 1
#         # Log traffic diagnostics to the screen every 2 seconds
#         self.get_logger().info(
#             f"[DIAGNOSTIC] Raw frames seen -> Depth: {self.depth_received_count} | RGB: {self.rgb_received_count}. "
#             f"If synchronization remains broken, compare timestamps.", 
#             throttle_duration_sec=2.0
#         )

#     def get_guided_row_profile(self, depth_matrix, rgb_matrix, center_row_idx, width):
#         half_w = self.v_window // 2
#         start_row = max(0, center_row_idx - half_w)
#         end_row = min(depth_matrix.shape[0], center_row_idx + half_w + 1)
        
#         depth_block = depth_matrix[start_row:end_row, :]
#         rgb_block = rgb_matrix[start_row:end_row, :, :]
        
#         depth_scaled = np.clip(depth_block * 100.0, 0, 255).astype(np.uint8)
        
#         filtered_depth_block = cv2.bilateralFilter(
#             src=depth_scaled,
#             d=self.bilateral_diameter,
#             sigmaColor=self.sigma_color,
#             sigmaSpace=self.sigma_space
#         )
        
#         depth_block_meters = filtered_depth_block.astype(np.float32) / 100.0
        
#         valid_mask = (depth_block_meters > 0.1)
#         with np.errstate(all='ignore'):
#             block_with_nans = np.where(valid_mask, depth_block_meters, np.nan)
#             all_nan_cols = np.all(np.isnan(block_with_nans), axis=0)
#             composite_row = np.nanmedian(block_with_nans, axis=0)
#             composite_row[all_nan_cols] = 0.0
            
#         composite_row = np.nan_to_num(composite_row, nan=0.0)
        
#         composite_row_matrix = composite_row.reshape(1, width)
#         final_clean_row = cv2.medianBlur(composite_row_matrix, 5).flatten()
#         return final_clean_row

#     def synchronized_callback(self, depth_msg: Image, rgb_msg: Image):
#         # Confirm synchronization was achieved
#         self.get_logger().info("--> SUCCESS: Frames Synchronized! Processing pipeline active.", throttle_duration_sec=5.0)

#         try:
#             cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
#             cv_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
#         except Exception as e:
#             self.get_logger().error(f"Failed to parse frames: {str(e)}")
#             return

#         depth_meter = cv_depth.astype(np.float32) / 1000.0
#         height, width = depth_meter.shape
        
#         if self.r70_consecutive_counts is None:
#             self.r70_consecutive_counts = np.zeros(width, dtype=np.int32)
#             self.r80_consecutive_counts = np.zeros(width, dtype=np.int32)

#         row1_idx = int(height * self.row1_pct)
#         row2_idx = int(height * self.row2_pct)
        
#         row1_data = self.get_guided_row_profile(depth_meter, cv_rgb, row1_idx, width)
#         row2_data = self.get_guided_row_profile(depth_meter, cv_rgb, row2_idx, width)

#         # --- STARTUP CALIBRATION PHASE ---
#         if not self.calibrated:
#             # Widen the calibration view to the middle 40% of the image to avoid tiny local dropouts
#             quarter_w = width // 4
#             r1_seg = row1_data[quarter_w : 3 * quarter_w]
#             r2_seg = row2_data[quarter_w : 3 * quarter_w]
            
#             # Extract valid values greater than 10cm
#             v_r1 = r1_seg[r1_seg > 0.1]
#             v_r2 = r2_seg[r2_seg > 0.1]
            
#             # If we have at least SOME valid floor data in this frame, use it!
#             if len(v_r1) > 5 and len(v_r2) > 5:
#                 self.row1_samples.append(np.median(v_r1))
#                 self.row2_samples.append(np.median(v_r2))
#                 self.get_logger().info(f"Calibrating floor baselines: Frame {len(self.row1_samples)}/{self.calibration_frames_needed}")
#             else:
#                 self.get_logger().warn("Calibration frame skipped: Too many invalid 0.0 values.", throttle_duration_sec=2.0)
#                 return
                
#             if len(self.row1_samples) >= self.calibration_frames_needed:
#                 self.row1_baseline = float(np.median(self.row1_samples))
#                 self.row2_baseline = float(np.median(self.row2_samples))
#                 self.calibrated = True
#                 self.get_logger().info(f"--> CALIBRATION COMPLETE! Locked: 70%={self.row1_baseline:.2f}m, 80%={self.row2_baseline:.2f}m")
#                 self.get_logger().info("Detection topics are now LIVE and publishing.")
#             else:
#                 return

#         # --- RADIAL DISTANCE EVALUATION ---
#         scan_70 = self.create_base_scan(depth_msg, width)
#         scan_80 = self.create_base_scan(depth_msg, width)

#         ranges_70 = [float('inf')] * width
#         ranges_80 = [float('inf')] * width
        
#         cx = width / 2.0
#         focal_length = cx / math.tan(self.hfov_rad / 2.0)
#         pixel_offsets = np.arange(width) - cx
#         cos_angles = np.cos(np.arctan(pixel_offsets / focal_length))

#         for i in range(width):
#             cos_a = cos_angles[i]
            
#             # Row 70%
#             r1_dist = float(row1_data[i])
#             r1_is_hazard = False
#             if r1_dist > 0.0:
#                 if (r1_dist > (self.row1_baseline + self.ditch_threshold) and r1_dist <= self.max_detection_range) or \
#                    (r1_dist < (self.row1_baseline - self.obstacle_threshold)):
#                     r1_is_hazard = True
            
#             if r1_is_hazard:
#                 self.r70_consecutive_counts[i] += 1
#             else:
#                 self.r70_consecutive_counts[i] = 0
                
#             if self.r70_consecutive_counts[i] >= self.history_depth:
#                 ranges_70[i] = r1_dist / cos_a

#             # Row 80%
#             r2_dist = float(row2_data[i])
#             r2_is_hazard = False
#             if r2_dist > 0.0:
#                 if (r2_dist > (self.row2_baseline + self.ditch_threshold) and r2_dist <= self.max_detection_range) or \
#                    (r2_dist < (self.row2_baseline - self.obstacle_threshold)):
#                     r2_is_hazard = True
                    
#             if r2_is_hazard:
#                 self.r80_consecutive_counts[i] += 1
#             else:
#                 self.r80_consecutive_counts[i] = 0
                
#             if self.r80_consecutive_counts[i] >= self.history_depth:
#                 ranges_80[i] = r2_dist / cos_a

#         scan_70.ranges = ranges_70
#         scan_80.ranges = ranges_80
        
#         self.pub_70.publish(scan_70)
#         self.pub_80.publish(scan_80)

#     def create_base_scan(self, msg, width):
#         scan = LaserScan()
#         scan.header.stamp = self.get_clock().now().to_msg()
#         scan.header.frame_id = msg.header.frame_id
#         scan.angle_min = -(self.hfov_rad / 2.0)
#         scan.angle_max = (self.hfov_rad / 2.0)
#         scan.angle_increment = (scan.angle_max - scan.angle_min) / (width - 1)
#         scan.range_min = 0.1
#         scan.range_max = 10.0
#         return scan

# def main(args=None):
#     rclpy.init(args=args)
#     node = RGBDGuidedDepthDetector()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

####################----------------------TWO LINES Solution with filtering techniques
# import rclpy
# from rclpy.node import Node
# import numpy as np
# import cv2
# import math
# from cv_bridge import CvBridge

# from sensor_msgs.msg import Image, LaserScan
# from rclpy.qos import qos_profile_sensor_data

# class BlockFilteredDepthDetector(Node):
#     def __init__(self):
#         super().__init__('block_filtered_depth_detector')
        
#         self.bridge = CvBridge()
        
#         # --- PARAMETERS ---
#         self.input_depth_topic = '/camera/camera/depth/image_rect_raw'
#         self.output_scan_65_topic = '/detected_ditches_65'
#         self.output_scan_75_topic = '/detected_ditches_75'
        
#         # Slightly expanded thresholds to ignore minor floor variances
#         self.ditch_threshold = 0.09     # 9cm drop
#         self.obstacle_threshold = 0.07  # 7cm rise
#         self.max_detection_range = 2.00 
#         self.hfov_rad = 1.50  
        
#         self.row1_pct = 0.70           
#         self.row2_pct = 0.80            #0.75  
#         self.v_window = 9 
        
#         ####  0.7 & 0.8 (sometime very close to object, can collide), 0.65 & 0.7 (good combination for distanct detection but sometime gives little noise)

#         # --- STARTUP CALIBRATION VARIABLES ---
#         self.calibrated = False
#         self.calibration_frames_needed = 30  
#         self.row1_samples = []
#         self.row2_samples = []
        
#         self.row1_baseline = 0.0
#         self.row2_baseline = 0.0
        
#         # --- FIX: TEMPORAL HISTORY BUFFERS ---
#         # We track how many consecutive frames an obstacle has been seen at each horizontal index (column)
#         self.history_depth = 5  # Must be verified over 5 frames  ///  Increase to reduce more noise but it also increase the letancy 
#         self.r65_consecutive_counts = None
#         self.r75_consecutive_counts = None
        
#         # --- SUBSCRIPTIONS & PUBLISHERS ---
#         self.subscription = self.create_subscription(
#             Image,
#             self.input_depth_topic,
#             self.depth_callback,
#             qos_profile_sensor_data)
            
#         self.pub_65 = self.create_publisher(LaserScan, self.output_scan_65_topic, 10)
#         self.pub_75 = self.create_publisher(LaserScan, self.output_scan_75_topic, 10)
            
#         self.get_logger().info("Node started with Spatial 2D + Temporal Filter Layer.")

#     def get_clean_row_profile(self, depth_matrix, center_row_idx, width):
#         half_w = self.v_window // 2
#         start_row = max(0, center_row_idx - half_w)
#         end_row = min(depth_matrix.shape[0], center_row_idx + half_w + 1)
        
#         row_block = depth_matrix[start_row:end_row, :]
#         valid_mask = (row_block > 0.1)
        
#         with np.errstate(all='ignore'):
#             block_with_nans = np.where(valid_mask, row_block, np.nan)
#             all_nan_cols = np.all(np.isnan(block_with_nans), axis=0)
#             composite_row = np.nanmedian(block_with_nans, axis=0)
#             composite_row[all_nan_cols] = 0.0
            
#         composite_row = np.nan_to_num(composite_row, nan=0.0)
        
#         composite_row_matrix = composite_row.reshape(1, width)
#         final_clean_row = cv2.medianBlur(composite_row_matrix, 5).flatten()
        
#         return final_clean_row

#     def depth_callback(self, msg: Image):
#         try:
#             cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
#         except Exception as e:
#             self.get_logger().error(f"Failed to parse input depth image: {str(e)}")
#             return

#         depth_meter = cv_depth.astype(np.float32) / 1000.0
#         height, width = depth_meter.shape
        
#         # Initialize temporal history counters on the very first valid frame size
#         if self.r65_consecutive_counts is None:
#             self.r65_consecutive_counts = np.zeros(width, dtype=np.int32)
#             self.r75_consecutive_counts = np.zeros(width, dtype=np.int32)

#         row1_idx = int(height * self.row1_pct)
#         row2_idx = int(height * self.row2_pct)
        
#         row1_data = self.get_clean_row_profile(depth_meter, row1_idx, width)
#         row2_data = self.get_clean_row_profile(depth_meter, row2_idx, width)

#         # --- STARTUP CALIBRATION PHASE ---
#         if not self.calibrated:
#             center_index = width // 2
#             r1_seg = row1_data[center_index - 10 : center_index + 10]
#             r2_seg = row2_data[center_index - 10 : center_index + 10]
#             v_r1 = r1_seg[r1_seg > 0.1]
#             v_r2 = r2_seg[r2_seg > 0.1]
            
#             if len(v_r1) == 0 or len(v_r2) == 0:
#                 return
                
#             self.row1_samples.append(np.median(v_r1))
#             self.row2_samples.append(np.median(v_r2))
                
#             if len(self.row1_samples) >= self.calibration_frames_needed:
#                 self.row1_baseline = float(np.median(self.row1_samples))
#                 self.row2_baseline = float(np.median(self.row2_samples))
#                 self.calibrated = True
#                 self.get_logger().info(f"--> CALIBRATION READY! Locked: 65%={self.row1_baseline:.2f}m, 75%={self.row2_baseline:.2f}m")
#             else:
#                 return

#         # --- PROCESSING WITH TEMPORAL FILTER ---
#         scan_65 = self.create_base_scan(msg, width)
#         scan_75 = self.create_base_scan(msg, width)

#         ranges_65 = [float('inf')] * width
#         ranges_75 = [float('inf')] * width
        
#         cx = width / 2.0
#         focal_length = cx / math.tan(self.hfov_rad / 2.0)
#         pixel_offsets = np.arange(width) - cx
#         cos_angles = np.cos(np.arctan(pixel_offsets / focal_length))

#         for i in range(width):
#             cos_a = cos_angles[i]
            
#             # --- EVALUATE ROW 1 (65%) ---
#             r1_dist = float(row1_data[i])
#             r1_is_hazard = False
#             if r1_dist > 0.0:
#                 if (r1_dist > (self.row1_baseline + self.ditch_threshold) and r1_dist <= self.max_detection_range) or \
#                    (r1_dist < (self.row1_baseline - self.obstacle_threshold)):
#                     r1_is_hazard = True
            
#             if r1_is_hazard:
#                 self.r65_consecutive_counts[i] += 1
#             else:
#                 self.r65_consecutive_counts[i] = 0 # Instantly reset if it goes away for 1 single frame
                
#             # Only publish if it has survived the temporal filter window
#             if self.r65_consecutive_counts[i] >= self.history_depth:
#                 ranges_65[i] = r1_dist / cos_a

#             # --- EVALUATE ROW 2 (75%) ---
#             r2_dist = float(row2_data[i])
#             r2_is_hazard = False
#             if r2_dist > 0.0:
#                 if (r2_dist > (self.row2_baseline + self.ditch_threshold) and r2_dist <= self.max_detection_range) or \
#                    (r2_dist < (self.row2_baseline - self.obstacle_threshold)):
#                     r2_is_hazard = True
                    
#             if r2_is_hazard:
#                 self.r75_consecutive_counts[i] += 1
#             else:
#                 self.r75_consecutive_counts[i] = 0
                
#             if self.r75_consecutive_counts[i] >= self.history_depth:
#                 ranges_75[i] = r2_dist / cos_a

#         scan_65.ranges = ranges_65
#         scan_75.ranges = ranges_75
        
#         self.pub_65.publish(scan_65)
#         self.pub_75.publish(scan_75)

#     def create_base_scan(self, msg, width):
#         scan = LaserScan()
#         scan.header.stamp = self.get_clock().now().to_msg()
#         scan.header.frame_id = msg.header.frame_id
#         scan.angle_min = -(self.hfov_rad / 2.0)
#         scan.angle_max = (self.hfov_rad / 2.0)
#         scan.angle_increment = (scan.angle_max - scan.angle_min) / (width - 1)
#         scan.range_min = 0.1
#         scan.range_max = 10.0
#         return scan

# def main(args=None):
#     rclpy.init(args=args)
#     node = BlockFilteredDepthDetector()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

#################-----------------TWO LINES Solution
# import rclpy
# from rclpy.node import Node
# import numpy as np
# import cv2
# import math
# from cv_bridge import CvBridge

# # ROS 2 message types
# from sensor_msgs.msg import Image, LaserScan
# from rclpy.qos import qos_profile_sensor_data

# class DualTopicDepthDetector(Node):
#     def __init__(self):
#         super().__init__('dual_topic_depth_detector')
        
#         self.bridge = CvBridge()
        
#         # --- PARAMETERS ---
#         self.input_depth_topic = '/camera/camera/depth/image_rect_raw'
        
#         # Split outputs for the costmap
#         self.output_scan_65_topic = '/detected_ditches_65'
#         self.output_scan_75_topic = '/detected_ditches_75'
        
#         # Physical thresholds
#         self.ditch_threshold = 0.08     
#         self.obstacle_threshold = 0.06  
#         self.max_detection_range = 2.00 
        
#         # Camera Intrinsics 
#         self.hfov_rad = 1.50  
        
#         # --- CONFIGURATION FOR TWO TARGET ROWS ---
#         self.row1_pct = 0.65  # Row 1 (65%)
#         self.row2_pct = 0.75  # Row 2 (75%)
        
#         # --- STARTUP CALIBRATION VARIABLES ---
#         self.calibrated = False
#         self.calibration_frames_needed = 30  
#         self.row1_samples = []
#         self.row2_samples = []
        
#         self.row1_baseline = 0.0
#         self.row2_baseline = 0.0
        
#         # --- SUBSCRIPTIONS & PUBLISHERS ---
#         self.subscription = self.create_subscription(
#             Image,
#             self.input_depth_topic,
#             self.depth_callback,
#             qos_profile_sensor_data)
            
#         # Two independent publishers
#         self.pub_65 = self.create_publisher(LaserScan, self.output_scan_65_topic, 10)
#         self.pub_75 = self.create_publisher(LaserScan, self.output_scan_75_topic, 10)
            
#         self.get_logger().info("Dual-Topic Node Initialized. Separate streams tracking 65% and 75% rows.")

#     def depth_callback(self, msg: Image):
#         try:
#             cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
#         except Exception as e:
#             self.get_logger().error(f"Failed to parse input depth image: {str(e)}")
#             return

#         depth_meter = cv_depth.astype(np.float32) / 1000.0
#         height, width = depth_meter.shape
        
#         row1_idx = int(height * self.row1_pct)
#         row2_idx = int(height * self.row2_pct)
        
#         # Anti salt-and-pepper filters
#         r1_matrix = depth_meter[row1_idx, :].reshape(1, width)
#         row1_data = cv2.medianBlur(r1_matrix, 5).flatten()
        
#         r2_matrix = depth_meter[row2_idx, :].reshape(1, width)
#         row2_data = cv2.medianBlur(r2_matrix, 5).flatten()

#         # --- DYNAMIC STARTUP CALIBRATION PHASE ---
#         if not self.calibrated:
#             center_index = width // 2
            
#             v_r1 = row1_data[center_index - 10 : center_index + 10][row1_data[center_index - 10 : center_index + 10] > 0.1]
#             if len(v_r1) > 0: self.row1_samples.append(np.median(v_r1))
                
#             v_r2 = row2_data[center_index - 10 : center_index + 10][row2_data[center_index - 10 : center_index + 10] > 0.1]
#             if len(v_r2) > 0: self.row2_samples.append(np.median(v_r2))
                
#             if len(self.row1_samples) >= self.calibration_frames_needed and len(self.row2_samples) >= self.calibration_frames_needed:
#                 self.row1_baseline = float(np.median(self.row1_samples))
#                 self.row2_baseline = float(np.median(self.row2_samples))
#                 self.calibrated = True
#                 self.get_logger().info(f"--> CALIBRATION COMPLETE! Locked 65% ({self.row1_baseline:.2f}m) and 75% ({self.row2_baseline:.2f}m)")
#             else:
#                 return

#         # --- PREPARE BASE LASERSCAN MESSAGES ---
#         scan_65 = self.create_base_scan(msg, width)
#         scan_75 = self.create_base_scan(msg, width)

#         ranges_65 = [float('inf')] * width
#         ranges_75 = [float('inf')] * width
        
#         cx = width / 2.0
#         focal_length = cx / math.tan(self.hfov_rad / 2.0)
#         pixel_offsets = np.arange(width) - cx
#         cos_angles = np.cos(np.arctan(pixel_offsets / focal_length))

#         # --- PROCESS BOTH ROWS INDEPENDENTLY ---
#         for i in range(width):
#             cos_a = cos_angles[i]
            
#             # Row 1 (65% / Upper Edge Detection)
#             r1_dist = float(row1_data[i])
#             if r1_dist > 0.0:
#                 if (r1_dist > (self.row1_baseline + self.ditch_threshold) and r1_dist <= self.max_detection_range) or \
#                    (r1_dist < (self.row1_baseline - self.obstacle_threshold)):
#                     ranges_65[i] = r1_dist / cos_a

#             # Row 2 (75% / Lower Edge Detection)
#             r2_dist = float(row2_data[i])
#             if r2_dist > 0.0:
#                 if (r2_dist > (self.row2_baseline + self.ditch_threshold) and r2_dist <= self.max_detection_range) or \
#                    (r2_dist < (self.row2_baseline - self.obstacle_threshold)):
#                     ranges_75[i] = r2_dist / cos_a

#         # Publish both sets of independent lines
#         scan_65.ranges = ranges_65
#         scan_75.ranges = ranges_75
        
#         self.pub_65.publish(scan_65)
#         self.pub_75.publish(scan_75)

#     def create_base_scan(self, msg, width):
#         scan = LaserScan()
#         scan.header.stamp = self.get_clock().now().to_msg()
#         scan.header.frame_id = msg.header.frame_id
#         scan.angle_min = -(self.hfov_rad / 2.0)
#         scan.angle_max = (self.hfov_rad / 2.0)
#         scan.angle_increment = (scan.angle_max - scan.angle_min) / (width - 1)
#         scan.range_min = 0.1
#         scan.range_max = 10.0
#         return scan

# def main(args=None):
#     rclpy.init(args=args)
#     node = DualTopicDepthDetector()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

##############----------------CODE with 65% ROI -----------------------------
# import rclpy
# from rclpy.node import Node
# import numpy as np
# import cv2
# import math
# from cv_bridge import CvBridge

# # ROS 2 message types
# from sensor_msgs.msg import Image, LaserScan
# from rclpy.qos import qos_profile_sensor_data

# class DepthDitchDetector(Node):
#     def __init__(self):
#         super().__init__('depth_ditch_detector')
        
#         self.bridge = CvBridge()
        
#         # --- PARAMETERS ---
#         self.input_depth_topic = '/camera/camera/depth/image_rect_raw'
#         self.output_scan_topic = '/detected_ditches'  
        
#         # Physical camera specs 
#         self.ditch_threshold = 0.08     # 8cm drop below floor = Ditch
#         self.obstacle_threshold = 0.06  # 6cm rise above floor = Obstacle
#         self.max_detection_range = 2.00 
        
#         # Camera Intrinsics 
#         self.hfov_rad = 1.50  
        
#         # --- STARTUP CALIBRATION VARIABLES ---
#         self.calibrated = False
#         self.calibration_frames_needed = 30  
#         self.calibration_samples = []
#         self.real_flat_ground_distance = 0.0
        
#         # --- SUBSCRIPTIONS & PUBLISHERS ---
#         self.subscription = self.create_subscription(
#             Image,
#             self.input_depth_topic,
#             self.depth_callback,
#             qos_profile_sensor_data)
            
#         self.publisher = self.create_publisher(
#             LaserScan,
#             self.output_scan_topic,
#             10)
            
#         self.get_logger().info("Node initialized with Median Filter. Waiting for calibration...")

#     def depth_callback(self, msg: Image):
#         try:
#             cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
#         except Exception as e:
#             self.get_logger().error(f"Failed to parse input depth image: {str(e)}")
#             return

#         depth_meter = cv_depth.astype(np.float32) / 1000.0
#         height, width = depth_meter.shape
        
#         # Focus specifically on the 65% row
#         scan_row = int(height * 0.65) 
#         raw_row_data = depth_meter[scan_row, :]

#         # --- FIX: HORIZONTAL MEDIAN BLUR (ANTI SALT-AND-PEPPER) ---
#         # Reshape to a 2D image snippet so OpenCV can apply its fast median blur filter.
#         # A kernel size of 5 or 7 is great for wiping out individual pixel noise spikes.
#         row_matrix = raw_row_data.reshape(1, width)
#         filtered_matrix = cv2.medianBlur(row_matrix, 5)
#         row_data = filtered_matrix.flatten()

#         # --- ONE-TIME STARTUP CALIBRATION PHASE ---
#         if not self.calibrated:
#             center_index = width // 2
#             center_sample = row_data[center_index - 10 : center_index + 10]
#             valid_samples = center_sample[center_sample > 0.1] 
            
#             if len(valid_samples) > 0:
#                 self.calibration_samples.append(np.median(valid_samples))
                
#             if len(self.calibration_samples) >= self.calibration_frames_needed:
#                 self.real_flat_ground_distance = float(np.median(self.calibration_samples))
#                 self.calibrated = True
#                 self.get_logger().info(f"--> CALIBRATION COMPLETE! Locked baseline at: {self.real_flat_ground_distance:.3f} meters.")
#             else:
#                 return

#         # --- STANDARD DETECTION PROCESSING ---
#         scan_msg = LaserScan()
#         scan_msg.header.stamp = self.get_clock().now().to_msg()
#         scan_msg.header.frame_id = msg.header.frame_id
        
#         scan_msg.angle_min = -(self.hfov_rad / 2.0)
#         scan_msg.angle_max = (self.hfov_rad / 2.0)
#         scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (width - 1)
#         scan_msg.range_min = 0.1
#         scan_msg.range_max = 10.0

#         ranges = [float('inf')] * width
        
#         cx = width / 2.0
#         focal_length = cx / math.tan(self.hfov_rad / 2.0)

#         for i in range(width):
#             measured_dist = float(row_data[i])
            
#             if measured_dist == 0.0:
#                 continue

#             pixel_offset = i - cx
#             ray_angle = math.atan(pixel_offset / focal_length)
#             true_ray_dist = measured_dist / math.cos(ray_angle)
                
#             # 1. Check for DITCH
#             if measured_dist > (self.real_flat_ground_distance + self.ditch_threshold):
#                 if measured_dist <= self.max_detection_range:
#                     ranges[i] = true_ray_dist
            
#             # 2. Check for WALL/OBJECT
#             elif measured_dist < (self.real_flat_ground_distance - self.obstacle_threshold):
#                 ranges[i] = true_ray_dist

#         scan_msg.ranges = ranges
#         self.publisher.publish(scan_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = DepthDitchDetector()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

##############-------------FINAL WORKING-----------------------
# import rclpy
# from rclpy.node import Node
# import numpy as np
# import cv2
# import math
# from cv_bridge import CvBridge

# # ROS 2 message types
# from sensor_msgs.msg import Image, LaserScan
# from rclpy.qos import qos_profile_sensor_data

# class DepthDitchDetector(Node):
#     def __init__(self):
#         super().__init__('depth_ditch_detector')
        
#         self.bridge = CvBridge()
        
#         # --- PARAMETERS ---
#         self.input_depth_topic = '/camera/camera/depth/image_rect_raw'
#         self.output_scan_topic = '/detected_ditches'  
        
#         # Physical camera specs 
#         self.camera_height = 0.42  # Height of camera lens above flat ground (in meters)
#         self.ditch_threshold = 0.06  # Drops 6cm or more below floor = Ditch (Negative obstacle)
#         self.obstacle_threshold = 0.05  # Rises 5cm or more above floor = Obstacle (Positive obstacle)
        
#         # Camera Intrinsics 
#         # 1.50 rad (~86 deg) matches standard RealSense horizontal FOV configurations
#         self.hfov_rad = 1.50  
        
#         # --- SUBSCRIPTIONS & PUBLISHERS ---
#         self.subscription = self.create_subscription(
#             Image,
#             self.input_depth_topic,
#             self.depth_callback,
#             qos_profile_sensor_data)
            
#         self.publisher = self.create_publisher(
#             LaserScan,
#             self.output_scan_topic,
#             10)
            
#         self.get_logger().info("Initialized Optimized 2D Depth Ditch & Obstacle Detector Node.")

#     def depth_callback(self, msg: Image):
#         try:
#             cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
#         except Exception as e:
#             self.get_logger().error(f"Failed to parse input depth image: {str(e)}")
#             return

#         depth_meter = cv_depth.astype(np.float32) / 1000.0
#         height, width = depth_meter.shape
        
#         # Focus specifically on the lower half of the image
#         scan_row = int(height * 0.7) 
#         row_data = depth_meter[scan_row, :]

#         # middle_pixel_dist = row_data[width // 2]                    #   real_flat_ground_distance calculated by it
#         # self.get_logger().info(f"LIVE GROUND CHECK -> Center Pixel Distance: {middle_pixel_dist:.3f} meters")


#         # Create a standard LaserScan message structure
#         scan_msg = LaserScan()
#         scan_msg.header.stamp = self.get_clock().now().to_msg()
#         scan_msg.header.frame_id = msg.header.frame_id
        
#         # Standardize the target laser scan range mapping
#         scan_msg.angle_min = -(self.hfov_rad / 2.0)
#         scan_msg.angle_max = (self.hfov_rad / 2.0)
#         scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (width - 1)
#         scan_msg.range_min = 0.1
#         scan_msg.range_max = 10.0

#         ranges = [float('inf')] * width
#         real_flat_ground_distance = 0.770 
#         max_detection_range = 1.50
        
#         # Find the center pixel and focal length based on camera intrinsics
#         cx = width / 2.0
#         focal_length = cx / math.tan(self.hfov_rad / 2.0)

#         for i in range(width):
#             measured_dist = float(row_data[i])
            
#             # Skip invalid pixels (0.0) 
#             if measured_dist == 0.0:
#                 continue

#             # FIX 1: Compute the precise ray angle based on the pinhole camera matrix
#             pixel_offset = i - cx
#             ray_angle = math.atan(pixel_offset / focal_length)
            
#             # FIX 2: Project the perpendicular Z-depth out to the true hypotenuse range
#             true_ray_dist = measured_dist / math.cos(ray_angle)
                
#             # 1. Check for DITCH (Negative Obstacle)
#             if measured_dist > (real_flat_ground_distance + self.ditch_threshold):
#                 if measured_dist <= max_detection_range:
#                     ranges[i] = true_ray_dist
            
#             # 2. Check for WALL/OBJECT (Positive Obstacle)
#             elif measured_dist < (real_flat_ground_distance - self.obstacle_threshold):
#                 ranges[i] = true_ray_dist

#         scan_msg.ranges = ranges
#         self.publisher.publish(scan_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = DepthDitchDetector()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()




#####----------PADDING of Pixels ON LEFT SIDE -----------------------

# import rclpy
# from rclpy.node import Node
# import numpy as np
# import cv2
# from cv_bridge import CvBridge

# # ROS 2 message types
# from sensor_msgs.msg import Image, LaserScan
# from rclpy.qos import qos_profile_sensor_data

# class ObstacleDetector(Node):
#     def __init__(self):
#         super().__init__('obstacle_detector')
        
#         self.bridge = CvBridge()
        
#         # --- PARAMETERS ---
#         self.input_depth_topic = '/camera/camera/depth/image_rect_raw'
#         self.output_scan_topic = '/detected_ditches'  
        
#         # Physical camera specs 
#         self.camera_height = 0.42  # Height of camera lens above flat ground (in meters)
#         self.ditch_threshold = 0.06  # Drops 6cm or more below floor = Ditch (Negative obstacle)
#         self.obstacle_threshold = 0.05  # Rises 5cm or more above floor = Obstacle (Positive obstacle)
        
#         # --- SUBSCRIPTIONS & PUBLISHERS ---
#         self.subscription = self.create_subscription(
#             Image,
#             self.input_depth_topic,
#             self.depth_callback,
#             qos_profile_sensor_data)
            
#         self.publisher = self.create_publisher(
#             LaserScan,
#             self.output_scan_topic,
#             10)
            
#         self.get_logger().info("Initialized Optimized 2D Depth Ditch & Obstacle Detector Node.")

#     def depth_callback(self, msg: Image):
#         try:
#             cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
#         except Exception as e:
#             self.get_logger().error(f"Failed to parse input depth image: {str(e)}")
#             return

#         depth_meter = cv_depth.astype(np.float32) / 1000.0
#         height, width = depth_meter.shape
        
#         # Focus specifically on the lower half of the image
#         scan_row = int(height * 0.7) 
#         row_data = depth_meter[scan_row, :]

# #       middle_pixel_dist = row_data[width // 2]                    #   real_flat_ground_distance calculated by it
# #       self.get_logger().info(f"LIVE GROUND CHECK -> Center Pixel Distance: {middle_pixel_dist:.3f} meters")

#         # Create a standard LaserScan message structure
#         scan_msg = LaserScan()
#         scan_msg.header.stamp = self.get_clock().now().to_msg()
#         scan_msg.header.frame_id = msg.header.frame_id
        
#         scan_msg.angle_min = -0.75  # radians (~ -43 degrees)
#         scan_msg.angle_max = 0.75   # radians (~ +43 degrees)
#         scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (width - 1)
#         scan_msg.range_min = 0.1
#         scan_msg.range_max = 10.0

#         ranges = []
#         real_flat_ground_distance = 0.770 
#         max_detection_range = 1.50

#         for i in range(width):
#             measured_dist = float(row_data[i])
            
#             # Skip invalid pixels (0.0) 
#             if measured_dist == 0.0:
#                 ranges.append(float('inf'))
#                 continue
                
#             # 1. Check for DITCH (Negative Obstacle)
#             if measured_dist > (real_flat_ground_distance + self.ditch_threshold):
#                 if measured_dist <= max_detection_range:
#                     ranges.append(measured_dist)
#                 else:
#                     ranges.append(float('inf'))
            
#             # 2. Check for WALL/OBJECT (Positive Obstacle)
#             elif measured_dist < (real_flat_ground_distance - self.obstacle_threshold):
#                 ranges.append(measured_dist) 
                
#             # 3. Normal Flat Ground
#             else:
#                 ranges.append(float('inf')) 

#         # --- ADDED LOGIC: FILL INDEX 0 TO 21 IF OBSTACLE DETECTED ---
#         # Look at the left half of the frame (indices 22 to 61 in your dataset)
#         # Find the first valid numerical obstacle distance value
#         obstacle_sample = float('inf')
#         for val in ranges[22:100]:  # Scanning the window where your obstacle appears
#             if val != float('inf'):
#                 obstacle_sample = val
#                 break  # Grab the closest edge value and stop looking
        
#         # If an obstacle was found, overwrite indices 0 to 21 with that value
#         if obstacle_sample != float('inf'):
#             for idx in range(22):
#                 if idx < len(ranges):
#                     ranges[idx] = obstacle_sample

#         scan_msg.ranges = ranges
#         self.publisher.publish(scan_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = ObstacleDetector()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()



###########-----------------------------------------Working for both positive and Negative Obstacles-------------------------------------


# import rclpy
# from rclpy.node import Node
# import numpy as np
# import cv2
# from cv_bridge import CvBridge

# # ROS 2 message types
# from sensor_msgs.msg import Image, LaserScan
# from rclpy.qos import qos_profile_sensor_data

# class DepthDitchDetector(Node):
#     def __init__(self):
#         super().__init__('depth_ditch_detector')
        
#         self.bridge = CvBridge()
        
#         # --- PARAMETERS ---
#         self.input_depth_topic = '/camera/camera/depth/image_rect_raw'
#         self.output_scan_topic = '/detected_ditches'  
        
#         # Physical camera specs 
#         self.camera_height = 0.42  # Height of camera lens above flat ground (in meters)
#         self.ditch_threshold = 0.06  # Drops 6cm or more below floor = Ditch (Negative obstacle)
#         self.obstacle_threshold = 0.05  # Rises 5cm or more above floor = Obstacle (Positive obstacle)
        
#         # --- SUBSCRIPTIONS & PUBLISHERS ---
#         self.subscription = self.create_subscription(
#             Image,
#             self.input_depth_topic,
#             self.depth_callback,
#             qos_profile_sensor_data)
            
#         self.publisher = self.create_publisher(
#             LaserScan,
#             self.output_scan_topic,
#             10)
            
#         self.get_logger().info("Initialized Optimized 2D Depth Ditch & Obstacle Detector Node.")

#     def depth_callback(self, msg: Image):
#         try:
#             cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
#         except Exception as e:
#             self.get_logger().error(f"Failed to parse input depth image: {str(e)}")
#             return

#         depth_meter = cv_depth.astype(np.float32) / 1000.0
#         height, width = depth_meter.shape
        
#         # Focus specifically on the lower half of the image
#         scan_row = int(height * 0.7) 
#         row_data = depth_meter[scan_row, :]

#         # Create a standard LaserScan message structure
#         scan_msg = LaserScan()
#         scan_msg.header.stamp = self.get_clock().now().to_msg()
#         scan_msg.header.frame_id = msg.header.frame_id
        
#         scan_msg.angle_min = -0.75  # radians (~ -43 degrees)
#         scan_msg.angle_max = 0.75   # radians (~ +43 degrees)
#         scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (width-1)
#         scan_msg.range_min = 0.1
#         scan_msg.range_max = 10.0

#         ranges = []
#         real_flat_ground_distance = 0.770 
#         max_detection_range = 1.50

#         for i in range(width):
#             measured_dist = float(row_data[i])
            
#             # Skip invalid pixels (0.0) 
#             if measured_dist == 0.0:
#                 ranges.append(float('inf'))
#                 continue
                
#             # 1. Check for DITCH (Negative Obstacle)
#             # Measured distance is further away than the floor
#             if measured_dist > (real_flat_ground_distance + self.ditch_threshold):
#                 if measured_dist <= max_detection_range:
#                     ranges.append(measured_dist)
#                 else:
#                     ranges.append(float('inf')) # Ignore drops beyond our care range
            
#             # 2. Check for WALL/OBJECT (Positive Obstacle)
#             # Measured distance is closer to the camera than the floor
#             elif measured_dist < (real_flat_ground_distance - self.obstacle_threshold):
#                 ranges.append(measured_dist) # Object detected!
                
#             # 3. Normal Flat Ground
#             else:
#                 ranges.append(float('inf')) 

#         scan_msg.ranges = ranges
#         self.publisher.publish(scan_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = DepthDitchDetector()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()




#######-------------------------WORKING CODE for Negative Obstacles Only-----------------------------------------

# import rclpy
# from rclpy.node import Node
# import numpy as np
# import cv2
# from cv_bridge import CvBridge

# # ROS 2 message types
# from sensor_msgs.msg import Image, LaserScan
# from rclpy.qos import qos_profile_sensor_data

# class DepthDitchDetector(Node):
#     def __init__(self):
#         super().__init__('depth_ditch_detector')
        
#         self.bridge = CvBridge()
        
#         # --- PARAMETERS ---
#         # Subscribes directly to your active 2D depth stream
#         self.input_depth_topic = '/camera/camera/depth/image_rect_raw'
#         self.output_scan_topic = '/detected_ditches'
        
#         # Physical camera specs (Adjust based on your hardware mount)
#         self.camera_height = 0.42  # Height of camera lens above flat ground (in meters)
#         self.ditch_threshold = 0.05  # Drops 5cm or more below expected floor level = Ditch
        
#         # --- SUBSCRIPTIONS & PUBLISHERS ---
#         # RealSense uses sensor_data QoS for high-rate video streams
#         self.subscription = self.create_subscription(
#             Image,
#             self.input_depth_topic,
#             self.depth_callback,
#             qos_profile_sensor_data)
            
#         # Publishing a lightweight LaserScan makes it easy for Nav2 costmaps to read
#         self.publisher = self.create_publisher(
#             LaserScan,
#             self.output_scan_topic,
#             10)
            
#         self.get_logger().info("Initialized Optimized 2D Depth Ditch Detector Node.")

#     def depth_callback(self, msg: Image):
#         try:
#             # Convert ROS Image to OpenCV 16-bit single-channel depth image (values in mm)
#             cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
#         except Exception as e:
#             self.get_logger().error(f"Failed to parse input depth image: {str(e)}")
#             return

#         # Convert to meters for easier physical calculations
#         depth_meter = cv_depth.astype(np.float32) / 1000.0

#         # Grab image dimensions (e.g., 480 x 848)
#         height, width = depth_meter.shape
        
#         # Focus specifically on the lower half of the image (the ground in front of the robot)
#         scan_row = int(height * 0.7) 
#         row_data = depth_meter[scan_row, :]

#         # middle_pixel_dist = row_data[width // 2]                    ####   real_flat_ground_distance calculated by it
#         # self.get_logger().info(f"LIVE GROUND CHECK -> Center Pixel Distance: {middle_pixel_dist:.3f} meters")

#         # Create a standard LaserScan message structure
#         scan_msg = LaserScan()

#         scan_msg.header.stamp = self.get_clock().now().to_msg()
#         scan_msg.header.frame_id = msg.header.frame_id
#         # scan_msg.header = msg.header
#         # scan_msg.header.frame_id = msg.header.frame_id
        
#         # Approximate field of view limits for a D435/D455 (~86 degrees horizontal FOV)
#         scan_msg.angle_min = -0.75  # radians (~ -43 degrees)
#         scan_msg.angle_max = 0.75   # radians (~ +43 degrees)
#         scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / width
#         scan_msg.range_min = 0.1
#         scan_msg.range_max = 10.0

#         ranges = []
        
#         real_flat_ground_distance = 0.770 
        
#         # 2. Filter out background noise. Since the floor is at 0.77m, 
#         # any reading beyond 1.5 meters is irrelevant for this specific row scan.
#         max_detection_range = 1.50

#         for i in range(width):
#             measured_dist = float(row_data[i])
            
#             # Skip invalid pixels (0.0) or noisy background objects way in the back
#             if measured_dist == 0.0 or measured_dist > max_detection_range:
#                 ranges.append(float('inf')) # Safe / Ignore
#                 continue
                
#             # Check if the floor drops away into a ditch
#             # (If measured distance is deeper than 0.77m + your 5cm threshold)
#             if measured_dist > (real_flat_ground_distance + self.ditch_threshold):
#                 ranges.append(measured_dist) # Real drop-off detected!
#             else:
#                 ranges.append(float('inf')) # Normal flat ground

#         scan_msg.ranges = ranges
#         self.publisher.publish(scan_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = DepthDitchDetector()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
