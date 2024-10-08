import time
import numpy as np
import rclpy
from rclpy.node import Node
import rclpy.time
from std_msgs.msg import String, Float64
from sensor_msgs.msg import FluidPressure, Temperature, NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
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

#region Consts
ORIGIN_ALT_MSL=616.56
#endregion Consts

class PreEkfTranslator(Node):
    def __init__(self):
        super().__init__('rome_ekf_wrapper')

        # last_assigned values:
        self.last_heading = 0.0
        self.last_pressure = None
        self.last_temp = None
        self.last_pres_header = None
        self.last_published_header : Header = None
        self.ekf_origin: GeoPointStamped = None
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


        self.get_logger().warning(f"publishing alt with noise = {self.get_parameter('generated_alt_noise_std').value}")

        self.subscription1 = self.create_subscription(
            Odometry, 
            '/mavros/global_position/local',
            self.global_position_local_callback,
            qos_profile_sensor_data
        )

        self.subscription3 = self.create_subscription(
            Odometry,
            '/odom/base_link',
            self.odom_gz_callback,
            10
        )

        self.ekf_origin_sub = self.create_subscription(GeoPointStamped, '/mavros/global_position/gp_origin', self.ekf_origin_callback, qos_profile_sensor_data)
        # Publisher
        self.alt_publisher = self.create_publisher(PoseWithCovarianceStamped, self.get_parameter('out_topic').value, qos_profile_sensor_data)
        self.odom_publisher = self.create_publisher(Odometry, '/odometry/gazebo', 10)
        self.gps_publisher = self.create_publisher(NavSatFix, '/gps/fix', qos_profile_sensor_data)

        # Services
        self.msg_interval_client = self.create_client(MessageInterval, '/mavros/set_message_interval')
        self.set_datum_client = self.create_client(SetDatum, '/navsat_transform/set_datum')

        # Timer to operate:
        # self.out_timer = self.create_timer(float(1/self.get_parameter('out_rate').value), self.process_and_republish)
        self.ekf_origin_set_msg_interval_timer = self.create_timer(self.get_parameter('ekf_origin_msg_interval_request_rate').value, self.set_msg_interval_ekf_origin_callback)

        # TFs
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # TF Broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Timer to publish the new TF
        self.timer = self.create_timer(float(1/self.get_parameter('out_rate').value), self.publish_local_body_tf)

    def global_position_local_callback(self, msg: Odometry):
        # TODO: this alt is relative to HOME. get current home alt and republish relative to the EKF_ORIGIN
        pose_for_alt = PoseWithCovarianceStamped()
        pose_for_alt.header = msg.header
        pose_for_alt.header.frame_id = 'odom'
        pose_for_alt.pose.pose.position.z = msg.pose.pose.position.z
        pose_for_alt.pose.covariance[14] = 1.0
        self.alt_publisher.publish(pose_for_alt)

    def odom_gz_callback(self, msg: Odometry):
        self.get_logger().info(f'Received from odom')
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'odom'
        std = 0.0
        # msg.pose.pose.position.x = float(msg.pose.pose.position.x + np.random.normal(0, std, 1))
        msg.pose.covariance[0] = float(msg.pose.covariance[0])
        msg.twist.covariance[0] = 0.0
        msg.twist.covariance[7] = 0.0
        self.odom_publisher.publish(msg)

    def set_msg_interval_ekf_origin_callback(self):
        req = MessageInterval.Request()
        req.message_id = 49
        req.message_rate = self.get_parameter('ekf_origin_msg_rate').value
        self.msg_interval_client.call_async(req)
        return

    def ekf_origin_callback(self, msg: GeoPointStamped):
        self.get_logger().error(f"datum - {msg}")
        req = SetDatum.Request()
        req.geo_pose.position = msg.position
        self.set_datum_client.call_async(req)
        self.get_logger().error("end call")
        self.ekf_origin = msg

    def publish_local_body_tf(self):
        try:
            # Get the original transform
            trans = self.tf_buffer.lookup_transform(self.get_parameter('odom_frame').value, self.get_parameter('base_link_frame').value, rclpy.time.Time())
            original_rotation = trans.transform.rotation
            
            # Extract the quaternion from the original transform
            original_quat = [
                original_rotation.x,
                original_rotation.y,
                original_rotation.z,
                original_rotation.w
            ]

            # Convert the quaternion to Euler angles
            original_rotation = R.from_quat(original_quat)
            euler = original_rotation.as_euler('xyz')

            # Extract the yaw
            roll, pitch, yaw = euler

            # Create a new quaternion with the same yaw but zero roll and pitch
            new_rotation_from_odom = R.from_euler('xyz', [0, 0, yaw])
            quat_from_odom = new_rotation_from_odom.as_quat()
            rotation_diff =  original_rotation * new_rotation_from_odom.inv()

            new_quat = rotation_diff.as_quat()

            # Create a new transform
            new_transform = TransformStamped()
            new_transform.header.stamp = self.get_clock().now().to_msg()
            new_transform.header.frame_id = self.get_parameter('odom_frame').value  # Make the original transform the parent
            new_transform.child_frame_id = self.get_parameter('base_link_stab_frame').value      # Name for the new frame

            # Set the translation (keeping the same position but setting z to 0)
            new_transform.transform.translation = trans.transform.translation      # Adjust as needed

            
            # Set the rotation to the new quaternion
            new_transform.transform.rotation.x = quat_from_odom[0]
            new_transform.transform.rotation.y = quat_from_odom[1]
            new_transform.transform.rotation.z = quat_from_odom[2]
            new_transform.transform.rotation.w = quat_from_odom[3]

            # Publish the new transform
            self.tf_broadcaster.sendTransform(new_transform)

        except Exception as e:
            self.get_logger().error(f"Could not get transform: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PreEkfTranslator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
