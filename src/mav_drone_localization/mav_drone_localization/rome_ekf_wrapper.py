import time
import numpy as np
import rclpy
from rclpy.node import Node
import rclpy.time
from std_msgs.msg import String, Float64
from sensor_msgs.msg import FluidPressure, Temperature, NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Vector3Stamped, TwistWithCovarianceStamped
import tf2_geometry_msgs
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
# Define WGS84 ellipsoid constants
a = 6378137.0           # Earth's semi-major axis in meters
f = 1 / 298.257223563    # Flattening
b = a * (1 - f)         # Semi-minor axis

ORIGIN_ALT_MSL=616.56
#endregion Consts

from pygeodesy.geoids import GeoidPGM


class EllipsoidHandler:
    def __init__(self):
        self._egm96 = GeoidPGM('/usr/share/GeographicLib/geoids/egm96-5.pgm', kind=-3)

    def geoid_height(self, lat, lon):
        """Calculates AMSL to ellipsoid conversion offset.
        Uses EGM96 data with 5' grid and cubic interpolation.
        The value returned can help you convert from meters 
        above mean sea level (AMSL) to meters above
        the WGS84 ellipsoid.

        If you want to go from AMSL to ellipsoid height, add the value.

        To go from ellipsoid height to AMSL, subtract this value.
        """
        return self._egm96.height(lat, lon)


    def geopoint_ellipsoid_to_amsl(self, geo: GeoPointStamped):
        geo_amsl = GeoPointStamped
        geo_amsl = geo
        geo_amsl.position.altitude = geo.position.altitude - self.geoid_height(geo.position.latitude, geo.position.longitude)
        return geo_amsl
    
class PreEkfTranslator(Node):
    def __init__(self):
        super().__init__('rome_ekf_wrapper')
        self.ellipsoid_handler = EllipsoidHandler()
        # last_assigned values:
        self.last_heading = 0.0
        self.last_pressure = None
        self.last_temp = None
        self.last_pres_header = None
        self.last_published_header : Header = None
        self.ekf_origin: GeoPointStamped = None
        self.rate_of_climb : float = 0.0

        # Declare parameters for input and output topics
        self.declare_parameter('heading_topic', '/mavros/global_position/compass_hdg')
        self.declare_parameter('pressure_topic', '/mavros/imu/static_pressure')
        self.declare_parameter('temp_topic', '/mavros/imu/temperature_baro')
        self.declare_parameter('pressure_cov', 10.0 ) # pascals
        self.declare_parameter('temp_cov', 0.2) # deg c*
        self.declare_parameter('heading_cov', 0.02 ) # degrees
        self.declare_parameter('generated_alt_noise_std', 0.0) # in m
        self.declare_parameter('out_topic', '/alt/ahrs2')
        self.declare_parameter('out_rate', 10)
        self.declare_parameter('ekf_origin_msg_interval_request_rate', 0.1)
        self.declare_parameter('ekf_origin_msg_rate', 0.1) # HZ
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_link_frame', 'base_link')
        self.declare_parameter('base_link_stab_frame', 'base_link_stab')


        self.get_logger().warning(f"publishing alt with noise = {self.get_parameter('generated_alt_noise_std').value}")

        self.subscription1 = self.create_subscription(
            PoseWithCovarianceStamped, 
            '/mavros/ahrs2/alt',
            self.ahrs2_callback,
            10
        )

        self.subscription2 = self.create_subscription(
            Odometry,
            '/odom/base_link',
            self.odom_gz_callback,
            10
        )
        self.subscription3 = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/raw/fix',
            self.fix_callback,
            qos_profile_sensor_data
        )

        self.subscription4 = self.create_subscription(
            TwistWithCovarianceStamped,
            '/optical_flow/twist_with_cov',
            self.optical_flow_callback,
            qos_profile_sensor_data
        )
        self.ekf_origin_sub = self.create_subscription(GeoPointStamped, '/mavros/global_position/gp_origin', self.ekf_origin_callback, qos_profile_sensor_data)
        # Publisher
        self.alt_publisher = self.create_publisher(PoseWithCovarianceStamped, self.get_parameter('out_topic').value, 10)
        self.odom_publisher = self.create_publisher(Odometry, '/odometry/gazebo', 10)
        self.gps_publisher = self.create_publisher(Odometry, '/odometry/gps', 10)
        self.velocities_publisher = self.create_publisher(TwistWithCovarianceStamped, '/twist/optical_and_roc', 10)

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

    def ahrs2_callback(self, msg: PoseWithCovarianceStamped):
        if self.ekf_origin is None:
            return 
        # AHRS2 is alt is relative to AMSL. republish relative to the EKF_ORIGIN
        msg.header.frame_id = self.get_parameter("map_frame").value
        msg.pose.pose.position.z = msg.pose.pose.position.z - self.ekf_origin.position.altitude
        self.alt_publisher.publish(msg)

    def optical_flow_callback(self, msg: TwistWithCovarianceStamped):
        # TODO covariances transform
        msg.header.frame_id = self.get_parameter('base_link_frame').value

        try:
            # Get the original transform
            trans = self.tf_buffer.lookup_transform('base_link', 'odom', rclpy.time.Time())
            lin_vel = Vector3Stamped()
            lin_vel.vector.x = msg.twist.twist.linear.x
            lin_vel.vector.y = msg.twist.twist.linear.y
            lin_vel.vector.z = self.rate_of_climb
            vec_lin_trans = tf2_geometry_msgs.do_transform_vector3(lin_vel, trans)
            msg.twist.twist.linear = vec_lin_trans.vector
        except:
            self.get_logger().error("cant_transform")

        twists_variance = 0.5
        msg.twist.covariance[0] = twists_variance
        msg.twist.covariance[7] = twists_variance
        msg.twist.covariance[14] = twists_variance

        self.velocities_publisher.publish(msg)

    def odom_gz_callback(self, msg: Odometry):
        self.get_logger().info(f'Received from odom')

        self.rate_of_climb = msg.twist.twist.linear.z

        msg.header.frame_id = 'map'
        msg.child_frame_id = 'base_link'
        try:
            # Get the original transform
            trans = self.tf_buffer.lookup_transform('base_link', 'map', rclpy.time.Time())
            lin_vel = Vector3Stamped()
            lin_vel.vector.x = msg.twist.twist.linear.x
            lin_vel.vector.y = msg.twist.twist.linear.y
            lin_vel.vector.z = msg.twist.twist.linear.z
            vec_lin_trans = tf2_geometry_msgs.do_transform_vector3(lin_vel, trans)
            msg.twist.twist.linear = vec_lin_trans.vector
        except:
            self.get_logger().error("cant_transform")

        self.odom_publisher.publish(msg)

    def set_msg_interval_ekf_origin_callback(self):
        req = MessageInterval.Request()
        req.message_id = 49
        req.message_rate = self.get_parameter('ekf_origin_msg_rate').value
        self.msg_interval_client.call_async(req)
        return

    def ekf_origin_callback(self, msg: GeoPointStamped):
        req = SetDatum.Request()
        req.geo_pose.position = msg.position
        self.set_datum_client.call_async(req)
        self.ekf_origin = self.ellipsoid_handler.geopoint_ellipsoid_to_amsl(msg)

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

    def fix_callback(self, msg: NavSatFix):
        if self.ekf_origin is None:
            return
        lat, lon, alt = msg.latitude, msg.longitude, msg.altitude
        lat0, lon0, alt0 = self.ekf_origin.position.latitude, self.ekf_origin.position.longitude, self.ekf_origin.position.altitude
        enu_coords = self.latlonalt_to_enu(lat, lon, alt, lat0, lon0, alt0)
        self.get_logger().warn(f"{enu_coords}")
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.get_parameter('map_frame').value

        odom.pose.pose.position.x = enu_coords[0]
        odom.pose.pose.position.y = enu_coords[1]
        odom.pose.pose.position.z = enu_coords[2]
        odom.pose.covariance[0] = 0.5
        odom.pose.covariance[7] = 0.5
        odom.pose.covariance[14] = 1.0

        # self.gps_publisher.publish(odom)



    def geodetic_to_ecef(self, lat, lon, alt):
        """Convert geodetic coordinates (lat, lon, alt) to ECEF coordinates (x, y, z)."""
        lat, lon = np.radians(lat), np.radians(lon)
        
        # Prime vertical radius of curvature
        N = a / np.sqrt(1 - (2*f - f**2) * np.sin(lat)**2)
        
        # ECEF coordinates
        x = (N + alt) * np.cos(lat) * np.cos(lon)
        y = (N + alt) * np.cos(lat) * np.sin(lon)
        z = (N * (1 - (2*f - f**2)) + alt) * np.sin(lat)
        
        return x, y, z

    def ecef_to_enu(self, x, y, z, x0, y0, z0, lat0, lon0):
        """Convert ECEF coordinates to ENU coordinates relative to a given origin (lat0, lon0, alt0)."""
        lat0, lon0 = np.radians(lat0), np.radians(lon0)
        
        # Differences between the point and the origin
        dx = x - x0
        dy = y - y0
        dz = z - z0
        
        # ENU transformation matrix
        t = np.array([
            [-np.sin(lon0), np.cos(lon0), 0],
            [-np.sin(lat0)*np.cos(lon0), -np.sin(lat0)*np.sin(lon0), np.cos(lat0)],
            [np.cos(lat0)*np.cos(lon0), np.cos(lat0)*np.sin(lon0), np.sin(lat0)]
        ])
        
        # Apply transformation
        enu = t @ np.array([dx, dy, dz])
        
        return enu

    def latlonalt_to_enu(self, lat, lon, alt, lat0, lon0, alt0):
        """Convert lat, lon, alt to ENU coordinates with respect to an origin point."""
        # Convert both origin and point coordinates to ECEF
        x0, y0, z0 = self.geodetic_to_ecef(lat0, lon0, alt0)
        x, y, z = self.geodetic_to_ecef(lat, lon, alt)
        
        # Convert ECEF to ENU
        enu = self.ecef_to_enu(x, y, z, x0, y0, z0, lat0, lon0)
        
        return enu

def main(args=None):
    rclpy.init(args=args)
    node = PreEkfTranslator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
