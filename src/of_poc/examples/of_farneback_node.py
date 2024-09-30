import math
import time
from collections import deque
from typing import List

import cv2 as cv
import cv_bridge
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Quaternion, TwistWithCovarianceStamped, TwistStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from rclpy.callback_groups import ReentrantCallbackGroup
from . import matrix_utils

class ImageToOF(Node):
    def __init__(self, image_topic: str):
        super().__init__("farneback_of_estimation_node")
        self.group = ReentrantCallbackGroup()
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, qos_profile=qos_profile_sensor_data, callback_group=self.group)
        # self.link_info_sub = self.create_subscription(
        #     LinkStates,
        #     "/gazebo/link_states",
        #     self.link_states_callback,
        #     qos_profile=qos_profile_sensor_data,
        #     callback_group=self.group
        # )
        self.camera_odometry_sub = self.create_subscription(Odometry, "/odom/fake_camera_link", self.camera_odometry_callback, qos_profile=qos_profile_sensor_data)
        self.twist_pub = self.create_publisher(TwistStamped, "/optical_flow/twist", 10)
        self.camera_orientation_queue: deque[Quaternion] = deque(maxlen=2)
        self.camera_height: float = 0.0

        self.camera_info: CameraInfo = None
        self.camera_link_name: str = "iris::iris_demo::my_gimbal::tilt_link"
        self.last_frame_time : float = 0.0
        self.last_frame_grayscale: np.ndarray = None
        self.anglular_fov_roi_deg = 20

    #####################################################################################
    #####        Comm callbacks                                     #####################
    #####################################################################################

    def image_callback(self, msg: Image) -> None:
        """
        Main function that drives the whole proccess.
        whenever an image message arrives, the OF is calculated and published.
        """
        dt = time.time() - self.last_frame_time
        self.last_frame_time = time.time()
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        cv_image = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)
        width, height = cv_image.shape[:2]
        cropped_image = self.crop_image_to_roi(cv_image, 50, 10)
        if self.last_frame_grayscale is not None:
            flow = self.calculate_full_flow_farneback(cropped_image)
            camera_movement_compensation_mat = self.get_camera_comp_mat(flow)

            fx, fy = self.get_flow_avg_vector(flow+camera_movement_compensation_mat)
            direction_unit_vector = matrix_utils.unit_vector_from_quaternion(matrix_utils.quaternion_msg_to_numpy(self.camera_orientation_queue[-1]))
            object_distance = self.camera_height / direction_unit_vector[2]
            vx, vy = self.translation_to_speed(fx, fy, object_distance, 50, 40, dt, width, height)
            print(f"{vx=}, {vy=}, {dt=}")
            self.publish_twist_msg(vx, vy)
        self.last_frame_grayscale = cropped_image
        print(f"{dt=}")
        return
    
    def link_states_callback(self, msg: LinkStates):
        """_summary_
        NOT USED
        """
        camera_orientation = None
        camera_height = None
        try:
            link_index = msg.name.index(self.camera_link_name)
            camera_orientation = msg.pose[link_index].orientation
            camera_height = msg.pose[link_index].position.z
        except ValueError or NameError as e:
            self.get_logger().error(f"Failed to pull camera orientation from Gazebo\nwith error {e}")

        finally:
            if camera_orientation:
                self.camera_orientation_queue.append(camera_orientation)
            if camera_height:
                self.camera_height = camera_height
        return

    def camera_odometry_callback(self, msg: Odometry):
        self.camera_orientation_queue.append(msg.pose.pose.orientation)
        self.camera_height = msg.pose.pose.position.z
        return

    def publish_twist_msg(self, vx, vy):
        msg = TwistStamped() # TwistWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        self.twist_pub.publish(msg)

    #####################################################################################
    ###        OF Functions                                         #####################
    #####################################################################################

    def calculate_full_flow_farneback(self, gray_scale_frame: np.ndarray) -> np.ndarray:
        # Calculates dense optical flow by Farneback method
        flow = cv.calcOpticalFlowFarneback(self.last_frame_grayscale, gray_scale_frame, None, 0.5, 3, 15, 3, 5, 1.2, 0)
        # print(flow)
        return flow

    def get_flow_avg_vector(self, flow: np.ndarray) -> np.ndarray:
        after_mean = np.mean(flow, axis=0)
        after_mean = np.mean(after_mean, axis=0)
        return after_mean

    def destroy_node(self):
        """
        Destroy the node as part of shutting down the component.
        """
        super().destroy_node()

    def get_camera_comp_mat(self, flow: np.ndarray) -> np.ndarray:
        """only the camera movement compensation"""
        previous_camera_orientation: Quaternion = self.camera_orientation_queue[0]
        current_camera_orientation: Quaternion = self.camera_orientation_queue[-1]
        relative_rot: Rotation = matrix_utils.relative_rotation_from_quaternions(previous_camera_orientation, current_camera_orientation)
        r, p, y = relative_rot.as_euler("zyx", degrees=False) # in radians
        # TODO: change rotation to camera tf is important! (howhowhowhow)
        

        # yaw_comp = y # in radians
        # pitch_comp = p # in radians
        
        comp_mat = np.zeros_like(flow)


        return comp_mat

    
    #####################################################################################
    ###        General Functions                                    #####################
    #####################################################################################

    def crop_image_to_roi(self, img: np.ndarray, img_fov_deg: float, desired_fov_deg: float) -> np.ndarray:
        height, width = img.shape[:2]
        crop_size = int(width * (desired_fov_deg / img_fov_deg))
        start_x = width // 2 - (crop_size // 2)
        start_y = height // 2 - (crop_size // 2)
        return img[start_y : start_y + crop_size, start_x : start_x + crop_size]

    def translation_to_speed(self, tx, ty, camera_center_distance, fovx_deg, fovy_deg, dtime, image_width, image_height) -> List[float]:
        half_x_meters = camera_center_distance * math.tan(math.radians(fovx_deg / 2))
        half_y_meters = camera_center_distance * math.tan(math.radians(fovy_deg / 2))
        vx = (half_x_meters * tx / image_width) / dtime
        vy = (half_y_meters * ty / image_height) / dtime
        return vx, vy
    
    # def run(self):
    #     while self.cap.isOpened():
    #         # time.sleep(0.3)
    #         self.num_frames += 1

    #         ret, frame = self.cap.read()

    #         gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    #         flow = self.calculate_full_flow_farneback(gray)
    #         if self.helper_mat is None:
    #             self.init_helper_mat(flow)
    #         # fps calculating
    #         elapsed_time = time.time() - self.start_time
    #         self.avg_fps = self.num_frames / elapsed_time
    #         print(f"{self.avg_fps=}")

    #         (flow_x, flow_y) = self.get_flow_avg_vector(flow)
    #         self.filtered_x = self.filtered_x * FILTER_FACTOR + flow_x * (1 - FILTER_FACTOR)
    #         self.filtered_y = self.filtered_y * FILTER_FACTOR + flow_y * (1 - FILTER_FACTOR)
    #         print(f"{flow_x, flow_y=}")
    #         vx, vy = self.translation_to_speed(
    #             flow_x,
    #             flow_y,
    #             80,
    #             50,
    #             40,
    #             1 / self.avg_fps,
    #             frame.shape[0],
    #             frame.shape[1],
    #         )
    #         print(f"{vx, vy=}")
    #         # print("filtered", self.filtered_x, self.filtered_y)
    #         self.draw_flow_representation_on_frame(frame, flow_x, flow_y, vis_factor=ARROW_VIS_FACTOR, title="average flow")
    #         self.draw_flow_representation_on_frame(
    #             frame,
    #             self.filtered_x,
    #             self.filtered_y,
    #             vis_factor=ARROW_VIS_FACTOR,
    #             start_point=(self.width // 3, self.height // 3),
    #             title="filtered flow",
    #         )
    #         # flow = np.ndarray(shape=(512, 640, 2))
    #         # flow.fill(1)
    #         curl = self.guess_camera_move_type(flow)
    #         self.draw_flow_representation_on_frame(
    #             frame,
    #             curl,
    #             0,
    #             vis_factor=0.5,
    #             start_point=(2 * self.width // 3, 2 * self.height // 3),
    #             title="curl",
    #         )
    #         print(f"{curl=}")

    #         self.filtered_curl = self.filtered_curl * FILTER_FACTOR + curl * (1 - FILTER_FACTOR)
    #         self.draw_flow_representation_on_frame(
    #             frame,
    #             self.filtered_curl,
    #             0,
    #             vis_factor=0.5,
    #             start_point=(int(2.5 * self.width) // 3, int(2.5 * self.height) // 3),
    #             title="filtered curl",
    #         )
    #         self.sum_curl = self.sum_curl + self.filtered_curl
    #         # print(flow.shape)
    #         cv.imshow("dense optical flow", frame)

    #         self.last_frame_greyscale = gray
    #         if cv.waitKey(1) & 0xFF == ord("q"):
    #             break

    # def draw_flow_representation_on_frame(
    #     self,
    #     frame,
    #     flow_x,
    #     flow_y,
    #     vis_factor=1.0,
    #     start_point: Tuple[int] = None,
    #     title: str = "",
    # ) -> None:
    #     if not start_point:
    #         height, width = frame.shape[:2]
    #         center_x = int(width / 2)
    #         center_y = int(height / 2)
    #         start_point = (center_x, center_y)
    #     print(f"{flow_x, flow_y=}")
    #     end_point = (
    #         int(start_point[0] + flow_x * vis_factor),
    #         int(start_point[1] + flow_y * vis_factor),
    #     )
    #     color = (0, 0, 255)  # Red color in BGR
    #     thickness = 2
    #     tipLength = 0.1  # Length of the arrow tip relative to the arrow length
    #     title_point = (start_point[0], start_point[1] - 20)
    #     cv.circle(frame, start_point, 4, (0, 0, 0), -1)

    #     cv.arrowedLine(frame, start_point, end_point, color, thickness, tipLength=tipLength)
    #     cv.putText(frame, title, title_point, 1, 2, (255, 0, 0), 2)

    # def init_helper_mat(self, matrix):
    #     height, width, _ = matrix.shape
    #     print(matrix.shape)
    #     self.helper_mat = np.zeros_like(matrix)
    #     print(self.helper_mat.shape)
    #     center_point = (width // 2, height // 2)
    #     for y in range(height):
    #         for x in range(width):
    #             point_relative_to_center = (x - center_point[0], y - center_point[1])
    #             radius = np.linalg.norm(point_relative_to_center)
    #             if radius == 0.0:
    #                 radius = 100000000
    #             # if radius > min(height, width)//2:
    #             #     self.helper_mat[y][x][0] = 0
    #             #     self.helper_mat[y][x][1] = 0
    #             # else:
    #             self.helper_mat[y][x][0] = -point_relative_to_center[1] / radius**2
    #             self.helper_mat[y][x][1] = point_relative_to_center[0] / radius**2
    #     print(self.helper_mat)

    #     self.cap.release()
    #     cv.destroyAllWindows()


# The following frees up resources and
# closes all windows
def main():
    rclpy.init()
    of = ImageToOF('gimbal_camera/image_raw')
    executor = MultiThreadedExecutor()
    executor.add_node(of)
    executor.spin()

    of.destroy_node()
    executor.shutdown()
    rclpy.shutdown()
    

if __name__ == "__main__":
    main()
