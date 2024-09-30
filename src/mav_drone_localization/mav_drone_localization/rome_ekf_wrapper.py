import time
import numpy as np
import rclpy
from rclpy.node import Node
import rclpy.time
from std_msgs.msg import String, Float64
from sensor_msgs.msg import FluidPressure, Temperature, NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped
from geographic_msgs.msg import GeoPointStamped
from mavros_msgs.srv import MessageInterval
from robot_localization.srv import SetDatum
import math
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header
from tf_transformations import quaternion_from_euler
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
#region Consts
ORIGIN_ALT_MSL=616.56
#endregion Consts

def yaw_ned_to_quaternion_msg_enu(yaw_degrees):
    # Adjust yaw so that 0 degrees points North, but quaternion is in ENU frame
    adjusted_yaw_degrees = 90.0 - yaw_degrees
    
    # Convert adjusted yaw from degrees to radians
    yaw_radians = math.radians(adjusted_yaw_degrees)

    # Create quaternion from Euler angles (roll, pitch, yaw)
    # In ENU, roll (x-axis) and pitch (y-axis) are 0, and yaw (z-axis) is the rotation
    q = quaternion_from_euler(0, 0, yaw_radians)

    # Create and return a Quaternion ROS message
    quaternion_msg = Quaternion()
    quaternion_msg.x = q[0]
    quaternion_msg.y = q[1]
    quaternion_msg.z = q[2]
    quaternion_msg.w = q[3]
    
    return quaternion_msg


def calculate_altitude_and_variance(pressure_pa, temperature_c, pressure_variance, temperature_variance) -> list[float]:
    # Constants
    T0 = 288.15  # Standard temperature at sea level in Kelvin
    L = 0.0065   # Temperature lapse rate in K/m
    P0 = 101325  # Standard pressure at sea level in Pa
    R = 8.31432  # Universal gas constant for air in J/(mol·K)
    g = 9.80665  # Acceleration due to gravity in m/s²
    M = 0.0289644  # Molar mass of Earth's air in kg/mol

    # Convert temperature to Kelvin
    temperature_k = temperature_c + 273.15

    # Calculate exponent for the pressure ratio
    exponent = (R * L) / (g * M)

    # Calculate the altitude using the barometric formula
    altitude = (temperature_k / L) * (1 - (pressure_pa / P0) ** exponent)

    # Calculate partial derivatives
    partial_h_P = (-temperature_k / L) * (pressure_pa / P0) ** exponent * (R * L) / (g * M) * (1 / pressure_pa)
    partial_h_T =  (1 / L) * (1 - (pressure_pa / P0) ** exponent)
    # Calculate altitude variance
    altitude_variance = (partial_h_P ** 2 * pressure_variance) + (partial_h_T ** 2 * temperature_variance)

    return altitude, altitude_variance

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

        self.get_logger().warning(f"publishing alt with noise = {self.get_parameter('generated_alt_noise_std').value}")

        # Subscribers
        self.subscription1 = self.create_subscription(
            Float64,
            self.get_parameter('heading_topic').value,
            self.heading_callback,
            qos_profile_sensor_data
        )
        self.subscription2 = self.create_subscription(
            FluidPressure,
            self.get_parameter('pressure_topic').value,
            self.pressure_callback,
            qos_profile_sensor_data
        )
        self.subscription3 = self.create_subscription(
            Temperature,
            self.get_parameter('temp_topic').value,
            self.temp_callback,
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
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, self.get_parameter('out_topic').value, qos_profile_sensor_data)
        self.odom_publisher = self.create_publisher(Odometry, '/odometry/gazebo', 10)
        self.gps_publisher = self.create_publisher(NavSatFix, '/gps/fix', qos_profile_sensor_data)

        # Services
        self.msg_interval_client = self.create_client(MessageInterval, '/mavros/set_message_interval')
        self.set_datum_client = self.create_client(SetDatum, '/navsat_transform/set_datum')

        # Timer to operate:
        self.out_timer = self.create_timer(float(1/self.get_parameter('out_rate').value), self.process_and_republish)
        self.ekf_origin_set_msg_interval_timer = self.create_timer(self.get_parameter('ekf_origin_msg_interval_request_rate').value, self.set_msg_interval_ekf_origin_callback)

    def heading_callback(self, msg: Float64):
        self.get_logger().info(f'Received from heading: {msg.data}')
        self.last_heading = msg.data


    def pressure_callback(self, msg: FluidPressure):
        self.get_logger().info(f'Received from pressure: {msg.fluid_pressure}')
        self.last_pressure = msg.fluid_pressure
        self.last_pres_header = msg.header

    def temp_callback(self, msg: Temperature):
        self.get_logger().info(f'Received from temp: {msg.temperature}')
        self.last_temp = msg.temperature

    def odom_gz_callback(self, msg: Odometry):
        self.get_logger().info(f'Received from odom')
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'base_link'
        std = 0.0
        msg.pose.pose.position.x = float(msg.pose.pose.position.x + np.random.normal(0, std, 1))
        msg.pose.covariance[0] = float(msg.pose.covariance[0])

        self.odom_publisher.publish(msg)

    def process_and_republish(self):
        # Simple example of processing: concatenate the string from topic1 with the float from topic2
        if not self.last_heading or not self.last_pressure or not self.last_temp or not self.last_pres_header:
            self.get_logger().info(f'hdg_and_alt output NOT Published')
            return
        
        pressure_var = self.get_parameter('pressure_cov').value
        temp_var = self.get_parameter('temp_cov').value
        alt_amsl, alt_var = calculate_altitude_and_variance(self.last_pressure, self.last_temp, pressure_var, temp_var)
        self.get_logger().info(f"{alt_amsl, alt_var=}")

        if self.last_published_header:
            if self.last_pres_header.stamp.nanosec == self.last_published_header.stamp.nanosec:
                self.get_logger().warning("same timestamp")
                return

        # Republish the processed data
        output_msg = PoseWithCovarianceStamped()
        output_msg.header = self.last_pres_header
        output_msg.header.frame_id = 'odom'
        output_msg.pose.pose.orientation = yaw_ned_to_quaternion_msg_enu(self.last_heading)
        output_msg.pose.covariance[35] = self.get_parameter('heading_cov').value # for yaw, in radians

        noise = float(np.random.normal(0,self.get_parameter('generated_alt_noise_std').value,1))
        if not self.ekf_origin:
            return
        else:
            origin_height = float(self.ekf_origin.position.altitude)
            self.get_logger().info(f"{origin_height=}")

        output_msg.pose.pose.position.z = 1.13*alt_amsl - origin_height # + noise
        output_msg.pose.covariance[14] = alt_var 
        self.publisher.publish(output_msg)
        
        self.last_published_header = self.last_pres_header
        
        self.get_logger().info(f'Published: "{output_msg}"')

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

def main(args=None):
    rclpy.init(args=args)
    node = PreEkfTranslator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
