import time
import numpy as np
import rclpy
from rclpy.node import Node
import rclpy.time
from std_msgs.msg import String, Float64
from sensor_msgs.msg import FluidPressure, Temperature, NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, TwistWithCovarianceStamped
from geographic_msgs.msg import GeoPointStamped
from mavros_msgs.srv import MessageInterval
# from mavros_msgs.msg import 
from robot_localization.srv import SetDatum
import math
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header
from tf_transformations import quaternion_from_euler
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
import tf2_ros
from scipy.spatial.transform import Rotation as R

class VisionEstimateNode(Node):
    def __init__(self):
        super().__init__('vision_estimate_node')

        self.last_published_header : Header = None

        # Declare parameters for input and output topics
        self.declare_parameter('heading_topic', '/mavros/global_position/compass_hdg')
        self.declare_parameter('pressure_topic', '/mavros/imu/static_pressure')
        self.declare_parameter('temp_topic', '/mavros/imu/temperature_baro')
        self.declare_parameter('pressure_cov', 10.0 ) # pascals
        self.declare_parameter('temp_cov', 0.2) # deg c*
        self.declare_parameter('heading_cov', 0.02 ) # degrees
        self.declare_parameter('generated_alt_noise_std', 0.0) # in m
        self.declare_parameter('out_topic', '~/hdg_and_alt')
        self.declare_parameter('out_rate', 10)
        self.declare_parameter('ekf_origin_msg_interval_request_rate', 0.1)
        self.declare_parameter('ekf_origin_msg_rate', 0.1) # HZ
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_link_frame', 'base_link')
        self.declare_parameter('base_link_stab_frame', 'base_link_stab')

        self.subscription = self.create_subscription(
            Odometry,
            '/odom/base_link',
            self.odom_gz_callback,
            10
        )

        # Publisher
        self.speed_publisher = self.create_publisher(TwistWithCovarianceStamped, '/mavros/vision_speed/speed_twist_cov', 10)
        self.odom_publisher = self.create_publisher(Odometry, '/mavros/odometry/out', 10)

        # TFs
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # TF Broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)


    def odom_gz_callback(self, odom_msg: Odometry):
        self.get_logger().info(f'Received from odom')
        msg = TwistWithCovarianceStamped()

        msg.header.frame_id = 'map'
        msg.header.stamp = odom_msg.header.stamp

        msg.twist = odom_msg.twist

        self.speed_publisher.publish(msg)

        odom_msg.header = msg.header
        odom_msg.child_frame_id = 'base_link'
        self.odom_publisher.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VisionEstimateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
