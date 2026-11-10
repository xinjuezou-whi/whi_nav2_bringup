#!/usr/bin/env python3

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
