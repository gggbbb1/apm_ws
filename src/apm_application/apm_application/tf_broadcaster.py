import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from geometry_msgs.msg import TransformStamped
from tf2_ros.buffer import Buffer

from tf2_ros import TransformBroadcaster, TransformListener, StaticTransformBroadcaster, TFMessage
from scipy.spatial.transform import Rotation
from apm_application.pid import PID
from apm_application.consts import *
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega


class TFHandler(Node):
    def __init__(self):
        super().__init__('tf_handler')
        self.get_logger().info("tf_handler")
        self.static_tf_broadcast = StaticTransformBroadcaster(self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(1, self.send_static)
        self.odom_optic_subscriber = self.create_subscription(Odometry, CAMERA_PHYSICAL_ODOM_TOPIC, self.camera_link_tf_from_odom, 10)


    def camera_link_tf_from_odom(self, msg: Odometry):
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = CAMERA_LINK_FRAME

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

             # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message

        t.transform.rotation = msg.pose.pose.orientation
        #print("transform published: ", t)
        # Send the transformation
        self.tf_broadcaster.sendTransform(t)
        # print("sent")

    
    def send_static(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = CAMERA_LINK_FRAME
        t.child_frame_id = CAMERA_OPTICAL_FRAME

        t.transform.translation.x = 0.025
        t.transform.translation.y = 0.009
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.5
        t.transform.rotation.y = -0.5
        t.transform.rotation.z = 0.5
        t.transform.rotation.w = -0.5

        self.static_tf_broadcast.sendTransform(t)
        
def main(args=None):
    rclpy.init(args=args)
    tf_handler = TFHandler()
    rclpy.spin(tf_handler)
    tf_handler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()