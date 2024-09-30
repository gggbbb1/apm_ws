import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point, TwistStamped
from builtin_interfaces.msg import Time
from .consts import (
    CAMERA_FLU_ODOM_TOPIC,
    TWIST_OUT_TOPIC,
    POSE_ERROR_TEST_TOPIC
)

class OdomToPoseNode(Node):
    def __init__(self):
        super().__init__('test_twist_vs_gz')
        self.is_first_odom = True
        self.integrated_y = self.integrated_x = 0.0
        self.subscription = self.create_subscription(
            Odometry,
            CAMERA_FLU_ODOM_TOPIC,  # Adjust topic name according to your setup
            self.odom_callback,
            10)
        self.subscription = self.create_subscription(
            TwistStamped,
           TWIST_OUT_TOPIC,  # Adjust topic name according to your setup
            self.of_twist_callback,
            10)
        self.publisher = self.create_publisher(
            PoseStamped,
            POSE_ERROR_TEST_TOPIC,  # Publish to a topic named 'pose'
            10)
        self.last_twist_time:  Time = None
        self.x = self.y = 0
        
    def of_twist_callback(self, msg: TwistStamped):
        if self.last_twist_time is None:
            self.last_twist_time = msg.header.stamp
            return 
        
        dt = self.time_from_stamp_msg(msg.header.stamp) - self.time_from_stamp_msg(self.last_twist_time)
        print(f"dt = {dt}")
        self.integrated_x += msg.twist.linear.x * dt
        self.integrated_y += msg.twist.linear.y * dt
        self.last_twist_time = msg.header.stamp


        err_x = self.integrated_x - self.x
        err_y = self.integrated_y - self.y
          # Create PoseStamped message with fixed z (0)
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.pose.position = Point(x=err_x, y=err_y, z=0.0)


        # Publish the pose message
        self.publisher.publish(pose_msg)
        self.get_logger().info(f"Published pose: x={err_x}, y={err_y}, z=0.0")

    def odom_callback(self, msg):
        
        # Extract x, y from odometry message
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        if self.is_first_odom:
            print(f"first pose: {self.x}, {self.y}")
            self.integrated_x = self.x
            self.integrated_y = self.y
            self.is_first_odom = False

    def time_from_stamp_msg(self, stamp: Time) -> float:
        return float(stamp.sec + stamp.nanosec / 1e9)


def main(args=None):
    rclpy.init(args=args)
    odom_to_pose_node = OdomToPoseNode()
    rclpy.spin(odom_to_pose_node)
    odom_to_pose_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()