import threading
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from message_filters import ApproximateTimeSynchronizer, TimeSynchronizer, Subscriber
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer
from tf2_ros import TransformListener

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

class DataSynchronizerNode(Node):
    def __init__(self):
        super().__init__('data_synchronizer')
        self.image_sub = Subscriber(self, Image, '/gimbal_camera/image_raw')
        self.odom_sub = Subscriber(self, Odometry, '/odom/fake_camera_link')

        self.ts = ApproximateTimeSynchronizer([ self.odom_sub, self.image_sub], 10, slop=0.1)
        self.ts.registerCallback(self.callback)

        # Initialize TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
    @rate_checker
    def callback(self , odom_msg, image_msg):
        # Process synchronized messages
        self.get_logger().info("Received synchronized messages.")
        self.get_logger().info(f"Image timestamp: {image_msg.header.stamp}")
        self.get_logger().info(f"Odometry timestamp: {odom_msg.header.stamp}")
        self.get_logger().info(f"diff timestamp: {odom_msg.header.stamp - image_msg.header.stamp}")
        
        # Get transform from base_link to camera_link at image_msg timestamp
        try:
            transform = self.tf_buffer.lookup_transform('map_swd', 'camera_optical', odom_msg.header.stamp)
            self.get_logger().info(f"Transform from base_link to camera_link at image timestamp:\n{transform}")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"Failed to lookup transform: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DataSynchronizerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()