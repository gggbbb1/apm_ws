import math
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import List

import cv2 as cv
import cv_bridge
import matplotlib.pyplot as plt
import numpy as np
import rclpy
import tf2_geometry_msgs
import tf2_ros
from builtin_interfaces.msg import Time
from geometry_msgs.msg import (
    Pose,
    Quaternion,
    TransformStamped,
    TwistStamped,
    TwistWithCovarianceStamped,
    Vector3Stamped,
)
from message_filters import ApproximateTimeSynchronizer, Subscriber
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformListener, tf2
from tf2_ros.buffer import Buffer

from of_poc.consts import *

from . import matrix_utils


def rate_checker(func):
    interval = 1  # Interval for checking rate (in seconds)
    function_call_count = 0
    lock = threading.Lock()
    running = True

    def count_function_calls():
        nonlocal function_call_count, running
        while running:
            time.sleep(interval)
            with lock:
                current_rate = function_call_count / interval
                print(f"Current rate: {current_rate:.2f} Hz")
                function_call_count = 0

    count_thread = threading.Thread(target=count_function_calls)
    count_thread.daemon = True
    count_thread.start()

    def wrapper(*args, **kwargs):
        nonlocal function_call_count
        with lock:
            function_call_count += 1
        return func(*args, **kwargs)

    return wrapper


@dataclass
class SingleFrameData:
    time_frame: Time
    grayscale_frame: np.ndarray
    camera_odometry: Odometry
    transform_from_map: TransformStamped


@dataclass
class CameraInfoMinimizedData:
    K: np.ndarray
    distortion_coef: np.ndarray


class ImageToOF(Node):
    def __init__(self, image_topic: str):
        super().__init__("lucas_of_estimation_node")
        self.init_parameters()
        self.bridge = cv_bridge.CvBridge()
        self.camera_info_sub = self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self.camera_info_callback, 10)
        self.twist_pub = self.create_publisher(TwistStamped, TWIST_OUT_TOPIC, 10)
        self.real_twist_pub = self.create_publisher(TwistStamped, GROUND_TRUTH_TOPIC, 10)
        self.twist_with_cov_pub = self.create_publisher(TwistWithCovarianceStamped, TWIST_WITH_COV_OUT_TOPIC, 10)

        self.last_frame_camera_odometry: Quaternion = None
        self.camera_height: float = 0.0
        self.camera_info_data: CameraInfoMinimizedData = None

        ## TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_odom_time = 0.0
        self.current_odom_dt = None
        self.current_img_dt = None

        ## LK
        self.is_first_image = True
        self.feature_params = dict(
            maxCorners=int(
                np.ceil(
                    self.get_parameter(MAX_FULL_FRAME_FEATURES).value
                    / (
                        self.get_parameter(SEGMENT_AMOUNT_HEIGHT).value
                        * self.get_parameter(SEGMENT_AMOUNT_WIDTH).value
                    )
                )
            ),
            qualityLevel=self.get_parameter(LK_QUALITY_LEVEL).value,
            minDistance=self.get_parameter(MIN_DIST_BETWEEN_FEATURES).value,
            blockSize=7,
        )
        self.lk_params = dict(
            winSize=(15, 15), maxLevel=2, criteria=(cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03)
        )
        self.color = np.random.randint(0, 255, (self.get_parameter(MAX_FULL_FRAME_FEATURES).value + 10, 3))
        self.debug_mode = self.get_parameter(DEBUG_MODE).value
        self.camera_comp_vectors = None
        self.p0 = []
        self.min_features_for_segment = np.ceil(
            self.get_parameter(MIN_FEATURES_IN_FRAME).value
            / (self.get_parameter(SEGMENT_AMOUNT_HEIGHT).value * self.get_parameter(SEGMENT_AMOUNT_WIDTH).value)
        )
        # image segment
        self.masks = []

        # message synchronization:
        image_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=1
        )
        self.ts_image_sub = Subscriber(self, Image, image_topic, qos_profile=image_qos_profile)
        self.ts_odom_sub = Subscriber(self, Odometry, CAMERA_FLU_ODOM_TOPIC)
        self.ts = ApproximateTimeSynchronizer([self.ts_odom_sub, self.ts_image_sub], 10, slop=0.01)
        self.ts.registerCallback(self.run)
        self.last_run_data: SingleFrameData = None
        self.last_time_not_ros = 0.0

    def init_parameters(self):
        self.declare_parameter(DEBUG_MODE, True)
        self.declare_parameter(MIN_FEATURES_IN_FRAME, 30)
        self.declare_parameter(MAX_FULL_FRAME_FEATURES, 500)
        self.declare_parameter(MIN_DIST_BETWEEN_FEATURES, 5)
        self.declare_parameter(LK_QUALITY_LEVEL, 0.5)
        self.declare_parameter(SEGMENT_AMOUNT_WIDTH, 3)
        self.declare_parameter(SEGMENT_AMOUNT_HEIGHT, 3)
        self.declare_parameter(MIN_INTERVAL_TO_COMPUTE, 0.05)
        self.declare_parameter(DEFAULT_VAR_VALUE, 0.2)

    @rate_checker
    def run(self, odom_msg: Odometry, img_msg: Image) -> None:
        """
        Main function that drives the whole proccess.
        whenever an image message arrives, the OF is calculated and published.
        """
        if time.time() - self.last_time_not_ros < self.get_parameter(MIN_INTERVAL_TO_COMPUTE).value:
            return

        current_time_not_ros = time.time()

        this_run_time = odom_msg.header.stamp

        cropped_image, cropped_image_gray, width, height = self.extract_images_from_ros_msg(img_msg, 1.0)
        try:
            current_transform = self.tf_buffer.lookup_transform(MAP_Z_DOWN, CAMERA_LINK_OPTICAL_FRAME, this_run_time)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"Failed to lookup transform: {e}")
            return

        if self.last_run_data is None:
            self.last_run_data = SingleFrameData(
                time_frame=this_run_time,
                grayscale_frame=cropped_image_gray,
                camera_odometry=odom_msg,
                transform_from_map=current_transform,
            )
            self.last_time_not_ros = current_time_not_ros
            return

        last_time_not_ros = self.last_time_not_ros
        self.last_time_not_ros = current_time_not_ros

        self.assign_height_and_publish_ground_truth(odom_msg)

        if self.is_first_image:
            self.first_image_configurations(cropped_image, cropped_image_gray, width, height)

        if self.p0 == []:
            return

        all_frame_good_new, all_frame_good_old = self.run_LK_for_frame(cropped_image_gray, width, height)

        new_feature_points_on_the_ground = self.new_calculate_features_position_in_world(
            cropped_image_gray, all_frame_good_new, current_transform, self.camera_height
        )

        old_feature_points_on_the_ground = self.new_calculate_features_position_in_world(
            cropped_image_gray,
            all_frame_good_old,
            self.last_run_data.transform_from_map,
            self.last_run_data.camera_odometry.pose.pose.position.z,
        )

        flow_separated = new_feature_points_on_the_ground - old_feature_points_on_the_ground
        # flow_separated = flow_separated[np.linalg.norm(flow_separated, axis=1) < 1]

        flow_seperated_in_m_s = flow_separated /  float(current_time_not_ros - last_time_not_ros)
        flow_in_m_s = self.get_flow_avg_vector(flow_seperated_in_m_s)

        var_x, var_y = self.get_variances_from_flow_m_s(flow_seperated_in_m_s)

        if self.debug_mode:
            if len(new_feature_points_on_the_ground) > 0:
                self.get_logger().info(
                    f"Calculated Flow in m/s: {flow_in_m_s}\n Variance x axis: {var_x}\n Variance y axis: {var_y}"
                )
                # self.get_logger().info(
                #     f"{float(current_time_not_ros - last_time_not_ros)}"
                # )
            self.show_lucas(cropped_image, all_frame_good_new, all_frame_good_old, new_feature_points_on_the_ground)

        # velocities_in_map = self.transform_flow_to_map_frame(flow_in_m_s)
        self.publish_twist_msg(flow_in_m_s[1], flow_in_m_s[0], var_x, var_y)

        self.last_run_data = SingleFrameData(this_run_time, cropped_image_gray, odom_msg, current_transform)
        return

    def camera_info_callback(self, msg: CameraInfo):
        # Extract K matrix and reshape it to 3x3
        K_data = msg.k
        K = np.array(K_data).reshape((3, 3))

        # Extract distortion coefficients
        D_data = msg.d

        # For the plumb_bob model, directly use the coefficients
        distortion_coef = np.array(D_data)

        self.camera_info_data = CameraInfoMinimizedData(K=K, distortion_coef=distortion_coef)
    
    def get_variances_from_flow_m_s(self, flow_separeted_in_m_s: np.ndarray) -> tuple[float]:
        """generate covariance matrix in the shape of array[64]. the important parts: cov[0]= covariance in x, cov[7]=covariance in y
            we expect the flow to be the same for all points (only the linear motion of the camera), and therefore take the variance of the answers.

        Args:
            flow_separeted_in_m_s (np.ndarray): list of 2D vectors that express for each point on the ground - the flow in m/s of it.
        """
        variance_x, variance_y = np.var(flow_separeted_in_m_s, axis=0)
        if np.isnan(variance_x) or np.isnan(variance_y):
            self.get_logger().error("!!!!!!!!!!!!!!!")
            variance_y = self.get_parameter(DEFAULT_VAR_VALUE).value
            variance_x = self.get_parameter(DEFAULT_VAR_VALUE).value

        return variance_x, variance_y


    def init_masks(
        self,
        frame_grayscale: np.ndarray,
        masks_empty_array: list,
        width: int,
        width_segments: int,
        height: int,
        height_segments: int,
    ):
        # Calculate block dimensions
        block_height = height // height_segments
        block_width = width // width_segments

        # Create masks for each block
        for i in range(height_segments):
            for j in range(width_segments):
                # Calculate block boundaries
                x_start = j * block_width
                y_start = i * block_height
                x_end = (j + 1) * block_width
                y_end = (i + 1) * block_height

                # Create a mask for the current block
                mask = np.zeros_like(frame_grayscale[:, :])
                mask[y_start:y_end, x_start:x_end] = 255

                masks_empty_array.append(mask)

    def extract_images_from_ros_msg(self, msg: Image, crop_percentage: float) -> list:
        """takes a ROS Image message, returns a cropped image (by prcnt) and its grayscale version.

        Args:
            msg (Image): ROS message
            crop_percentage (float): percentage of the image to be cropped
        Returns:
            list: image, grayscale image, width, height
        """
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")

        # cropped_image = self.crop_image_by_prcnt(cv_image, crop_percentage)
        cropped_image_gray = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)

        height, width = cropped_image_gray.shape[:2]
        return cv_image, cropped_image_gray, width, height

    def run_LK_for_frame(self, frame, width, height) -> tuple[np.ndarray]:
        """
        takes a frame,
        """
        all_frame_good_new = np.empty((0, 2))
        all_frame_good_old = np.empty((0, 2))

        for i, mask in enumerate(self.masks):

            if self.p0[i] is None:
                self.p0[i] = cv.goodFeaturesToTrack(frame, mask=mask, **self.feature_params)
                self.get_logger().warning(f"No features to track for LK in segment {i}")
                continue
            try:
                p1, st, err = cv.calcOpticalFlowPyrLK(
                    self.last_run_data.grayscale_frame, frame, self.p0[i], None, **self.lk_params
                )  # AND operator with mask to search for optical flow only in the segment.
            except cv.error as e:
                self.get_logger().warning(f"No features to track for LK in segment {i}")
                continue

            if p1 is None:
                self.p0[i] = cv.goodFeaturesToTrack(frame, mask=mask, **self.feature_params)
                # there are no features to track, reinitialize the feature points
                self.get_logger().warning(f"No features to track for LK in segment {i}")
                continue

            # Select good points
            segment_good_new = p1[st == 1]
            segment_good_old = self.p0[i][st == 1]
            segment_good_new, segment_good_old = self.filter_points(
                width, height, mask, segment_good_new, segment_good_old
            )

            # print("for segment ", i, len(segment_good_new))
            if len(segment_good_new) < self.min_features_for_segment:
                self.get_logger().warning(
                    f"Under {self.min_features_for_segment} features detected in segment {i}, reinitializing the feature points"
                )
                self.p0[i] = cv.goodFeaturesToTrack(frame, mask=mask, **self.feature_params)
            else:
                self.p0[i] = segment_good_new.reshape(-1, 1, 2)
            # -------------------------------------
            # add this segment points to the full list
            all_frame_good_new = np.append(all_frame_good_new, segment_good_new, axis=0)
            all_frame_good_old = np.append(all_frame_good_old, segment_good_old, axis=0)

        return all_frame_good_new, all_frame_good_old

    def filter_points(self, width, height, mask, segment_good_new, segment_good_old):
        mask_bool = mask.astype(bool)
        points_to_filter = self.point_in_frame(segment_good_new, width, height)
        segment_good_new = segment_good_new[points_to_filter]
        segment_good_old = segment_good_old[points_to_filter]
        segment_filter = mask_bool[segment_good_new[:, 1].astype(int), segment_good_new[:, 0].astype(int)]
        segment_good_new = segment_good_new[segment_filter]
        segment_good_old = segment_good_old[segment_filter]
        return segment_good_new, segment_good_old

    def first_image_configurations(self, cropped_image, cropped_image_gray, width, height):
        self.init_masks(
            cropped_image_gray,
            self.masks,
            width,
            self.get_parameter(SEGMENT_AMOUNT_WIDTH).value,
            height,
            self.get_parameter(SEGMENT_AMOUNT_HEIGHT).value,
        )
        for mask in self.masks:
            self.good_features_for_mask(cropped_image_gray, mask, self.p0)

        self.colored_mask = np.zeros_like(cropped_image)
        self.is_first_image = False

    def good_features_for_mask(self, frame: np.ndarray, mask: np.ndarray, empty_points_array: list):
        """
        Find and add good features (cv.goodFeaturesToTrack) to each segment of the frame.
        """
        current_segment_p0 = cv.goodFeaturesToTrack(frame, mask=mask, **self.feature_params)
        if current_segment_p0 is None:
            # no features ==== 1 feature and it is np.nan
            empty_points_array.append(np.array([]))
        else:
            empty_points_array.append(current_segment_p0)

    def new_calculate_features_position_in_world(
        self,
        frame: np.ndarray,
        features_list: np.ndarray,
        transform_from_map_z_down: TransformStamped,
        camera_height: float,
    ) -> np.ndarray:
        """gets frame, list of features, distance from image center and camera orientation, and returns the 3D location of each feature in the world frame.
            frame origin is the camera itself.

        Args:
            frame (np.ndarray): image frame
            features_list (np.ndarray): list of 2d points in the image
            transform_from_map_z_down (TransformStamped): transform to camera from map (z down)
            fov_deg (float): _description_
            distance_from_image_center (float): in meters from the object in the image center

        Returns:
            np.ndarray: list of 2d points in the world frame on the ground
        """
        height, width = frame.shape[:2]
        # Example camera intrinsics and distortion coefficients
        # camera_frame_vectors = matrix_utils.pixels_to_unit_vectors_in_camera_frame(features_list, width, height, self.anglular_fov_roi_deg)
        # print(camera_frame_vectors.shape)
        camera_frame_vectors = matrix_utils.points_to_unit_vectors_with_distortion(
            features_list, self.camera_info_data.K, self.camera_info_data.distortion_coef
        )

        t_matrix = matrix_utils.transformation_matrix_from_transformation(transform_from_map_z_down)
        world_frame_unit_vectors = matrix_utils.apply_transformation_matrix(t_matrix, camera_frame_vectors)

        camera_center_vector = matrix_utils.apply_transformation_matrix(t_matrix, [[0, 0, 1]])
        angle_rad_bw_camera_and_down = matrix_utils.angle_between_vectors(camera_center_vector, [0, 0, 1])

        intersection_points = self.find_intersection_points(world_frame_unit_vectors, camera_height)
        return intersection_points[:, :2]

    def find_intersection_points(self, world_frame_unit_vectors, camera_height) -> np.ndarray:
        # Compute the parameter t
        t = camera_height / world_frame_unit_vectors[:, 2]
        # Compute the intersection point
        intersection_points = world_frame_unit_vectors * t[:, np.newaxis]
        return intersection_points

    def show_lucas(self, frame, good_new, good_old, camera_comp_vectors=None):
        img = None
        for i, (new, old) in enumerate(zip(good_new, good_old)):
            a, b = new.ravel()
            c, d = old.ravel()

            self.colored_mask = cv.line(
                self.colored_mask, (int(a), int(b)), (int(c), int(d)), self.color[i].tolist(), 2
            )

            frame = cv.circle(frame, (int(a), int(b)), 2, self.color[i].tolist(), -1)

        img = cv.add(frame, self.colored_mask)

        if img is not None:
            cv.imshow("frame", frame)
            cv.waitKey(1)

    def point_in_frame(self, points: np.ndarray, width: int, height: int) -> np.ndarray:
        """returns boolean value wether the point is in the frame or not.

        Args:
            points (np.ndarray): array of points - shape (n, 2)
            width (int): of frame width
            height (int): of frame

        Returns:
            np.ndarray: is the point in the frame(as array)
        """
        in_frame = (points[:, 0] >= 0) & (points[:, 0] < width) & (points[:, 1] >= 0) & (points[:, 1] < height)
        return in_frame

    def assign_height_and_publish_ground_truth(self, msg: Odometry):
        # self.camera_odometry_queue.append(msg)
        self.camera_height = msg.pose.pose.position.z

        # publish real_truth:
        tw = TwistStamped()  # TwistWithCovarianceStamped()
        tw.header.frame_id = BASE_OF_FRAME
        tw.header.stamp = self.get_clock().now().to_msg()
        tw.twist.linear.x = msg.twist.twist.linear.x
        tw.twist.linear.y = msg.twist.twist.linear.y
        self.real_twist_pub.publish(tw)
        return

    def publish_twist_msg(self, vx, vy, variance_x, variance_y):
        current_time = self.get_clock().now().to_msg()
        msg = TwistStamped()
        msg.header.frame_id = WORLD_FRAME
        msg.header.stamp = current_time
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        self.twist_pub.publish(msg)

        msg_with_cov = TwistWithCovarianceStamped()
        msg_with_cov.header.frame_id = WORLD_FRAME
        msg_with_cov.header.stamp = current_time
        msg_with_cov.twist.twist.linear.x = vx
        msg_with_cov.twist.twist.linear.y = vy
        msg_with_cov.twist.covariance[0] = variance_x
        msg_with_cov.twist.covariance[7] = variance_y
        self.twist_with_cov_pub.publish(msg_with_cov)


    def get_flow_avg_vector(self, flow: np.ndarray) -> np.ndarray:
        if len(flow) > 0:
            after_mean = np.mean(flow, axis=0)
            # after_mean = np.mean(after_mean, axis=0)
            return after_mean
        else:
            return np.array([0.0, 0.0])

    def destroy_node(self):
        """
        Destroy the node as part of shutting down the component.
        """
        super().destroy_node()

    def crop_image_by_prcnt(self, img: np.ndarray, crop_percentage: float) -> np.ndarray:
        height, width = img.shape[:2]

        # Calculate the dimensions of the cropped box
        crop_width = int(width * crop_percentage)
        crop_height = int(height * crop_percentage)
        # crop_width = crop_height
        # Calculate the cropping box coordinates
        x_start = (width - crop_width) // 2
        y_start = (height - crop_height) // 2
        x_end = x_start + crop_width
        y_end = y_start + crop_height

        # Crop the image
        cropped_img = img[y_start:y_end, x_start:x_end]

        return cropped_img


def main():
    rclpy.init()
    of = ImageToOF("/gimbal_camera/image_raw")
    rclpy.spin(of)

    of.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
