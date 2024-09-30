import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv_bridge
import cv2
import numpy as np
from rclpy.qos import qos_profile_sensor_data

class ImageToVideoConverter(Node):
    def __init__(self):
        super().__init__('image_to_video_converter')
        self.bridge = cv_bridge.CvBridge()
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, qos_profile=qos_profile_sensor_data)
        self.video_writer = None

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            if self.video_writer is None:
                self.init_video_writer(cv_image)
            self.video_writer.write(cv_image)
        except Exception as e:
            self.get_logger().error('Error processing image: %s' % str(e))

    def init_video_writer(self, image):
        try:
            base_folder = "/home/user/Projects/robot_localization_ws/of_poc/videos/new_versions/"
            height, width, _ = image.shape
            video_format = 'mp4'  # or any other video format supported by OpenCV
            video_filename = base_folder + 'iris_translation.' + video_format
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            fps = 20  # Frames per second
            self.video_writer = cv2.VideoWriter(
                video_filename, fourcc, fps, (width, height))
        except Exception as e:
            self.get_logger().error('Error initializing video writer: %s' % str(e))

    def destroy_node(self):
        if self.video_writer is not None:
            self.video_writer.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    image_to_video_converter = ImageToVideoConverter()
    rclpy.spin(image_to_video_converter)
    image_to_video_converter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
