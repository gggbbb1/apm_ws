import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv_bridge
import cv2
import numpy as np
from rclpy.qos import qos_profile_sensor_data
import math
from dataclasses import dataclass
from geometry_msgs.msg import TwistWithCovarianceStamped
FPS = 9


@dataclass
class OdomResponse:
    roll_rate: float
    roll_rate_error: float
    x_vel: float
    x_vel_error: float
    y_vel: float
    y_vel_error: float


class Matcher:
    def __init__(self):
        self.feature_params = dict(maxCorners=100, qualityLevel=0.3,
                                   minDistance=7, blockSize=7)

        # Parameters for Lucas-Kanade optical flow
        self.lk_params = dict(winSize=(15, 15), maxLevel=2, criteria=(
            cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
        self.after_first_setup = False
        self.feature_points = None

    def is_initialised(self) -> bool:
        return self.after_first_setup

    def first_setup(self, frame):
        self.old_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self.feature_points = cv2.goodFeaturesToTrack(
            self.old_gray, mask=None, **self.feature_params)
        self.after_first_setup = True

    def translation_to_speed(self, tx, ty, curr_height, fovx_deg, fovy_deg, dtime, image_width, image_height) -> [float, float]:
        half_x_meters = curr_height * math.tan(math.radians(fovx_deg/2))
        half_y_meters = curr_height * math.tan(math.radians(fovy_deg/2))
        vx = (half_x_meters * tx / image_width) / dtime
        vy = (half_y_meters * ty / image_height) / dtime
        return vx, vy

    def run_frame(self, frame, dt) -> OdomResponse:
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Calculate optical flow
        new_feature_points, st, err = cv2.calcOpticalFlowPyrLK(
            self.old_gray, frame_gray, self.feature_points, None, **self.lk_params)

        # Select good points
        good_new = new_feature_points[st == 1]
        good_old = self.feature_points[st == 1]

        # Compute the transformation matrix (homography) between consecutive frames
        M, _ = cv2.estimateAffinePartial2D(good_old, good_new)
        print(M)
        roll_angle = np.arctan2(M[1, 0], M[0, 0]) * 180 / np.pi
        # Extract translation in pixels (tx, ty) from the last column of the transformation matrix
        tx = M[0, 2]
        ty = M[1, 2]
        vx, vy = self.translation_to_speed(tx, ty, 80, 50, 40, dt, self.old_gray.shape[0], self.old_gray.shape[1])
        print("Roll Angle:", roll_angle)
        print("Translation (tx, ty):", tx, ty)
        print("Speed (vx, vy): ", vx, vy)

        # Update previous frame and points
        self.old_gray = frame_gray.copy()
        self.feature_points = good_new.reshape(-1, 1, 2)

        res = OdomResponse(roll_angle/dt, 0.0, vx, 0.0, vy, 0.0)
        return res


class ImageToOdometry(Node):
    def __init__(self, odom_creator: Matcher, image_topic: str):
        super().__init__('image_to_video_converter')
        self.bridge = cv_bridge.CvBridge()
        self.subscription = self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile=qos_profile_sensor_data)
        self.twist_pub = self.create_publisher(TwistWithCovarianceStamped, '/optical_flow/twist', 10)
        self.odom_creator = odom_creator
        self.last_frame_time = 0.0

    def image_callback(self, msg):
        dt = time.time() - self.last_frame_time
        self.last_frame_time = time.time()
        cv_image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough')
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        if not self.odom_creator.is_initialised():
            self.odom_creator.first_setup(cv_image)
        else:
            response: OdomResponse = self.odom_creator.run_frame(
                cv_image, dt)
            if response:
                self.publish_twist_msg(response)


    def publish_twist_msg(self, data: OdomResponse):
        msg = TwistWithCovarianceStamped()
        # msg.header.stamp = float(time.time())
        msg.twist.twist.linear.x = data.x_vel
        msg.twist.twist.linear.y = data.y_vel
        msg.twist.twist.angular.z = data.roll_rate
        self.twist_pub.publish(msg)

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    image_topic = "/camera/image_raw"
    odomgen = Matcher()
    rclpy.init(args=args)
    image_to_video_converter = ImageToOdometry(odomgen, image_topic)
    rclpy.spin(image_to_video_converter)
    image_to_video_converter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
