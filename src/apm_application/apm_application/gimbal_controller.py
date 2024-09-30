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

from tf2_ros import TransformBroadcaster, TransformListener
from scipy.spatial.transform import Rotation
from apm_application.pid import PID
from apm_application.consts import *
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega


class GimbalControl(Node):
    def __init__(self):
        super().__init__('gimbal_controller')
        self.get_logger().info("gimbal_controller")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
        msg = self.master.recv_match()
        self.timer = self.create_timer(0.1, self.run)
        self.yaw_pid = PID(YAW_P, YAW_I, YAW_D)
        self.keep_alive()

    def neutral_tf_creator(self) -> TransformStamped:
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = CAMERA_LINK_FRAME
        t.child_frame_id = OBJECT_FRAME

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        #print("transform published: ", t)
        # Send the transformation
        return t
    
    def keep_alive(self):
        msg = None
        while msg is None:
            try:
                msg = self.master.recv_match()
                print(msg)
                # if not msg:
                #     continue
                if msg.get_type() == 'COMMAND_ACK':
                    print(msg.to_dict())
                if msg.get_type() == 'HEARTBEAT':
                    print(msg.to_dict())
            except:
                print("Gg")
            time.sleep(0.1)

    def manager_set_pitchyaw(self, pitch = 0, yaw = 0, is_rates=False):
        """
        GIMBAL_MANAGER_SET_PITCHYAW ( #287 )
        https://github.com/adinkra-labs/mavros_feature_gimbal-protocol-v2-plugin/blob/gimbal-protocol-v2-plugin/mavros_extras/src/plugins/gimbal_control.cpp#L433
        """
        # | ardupilotmega.GIMBAL_MANAGER_FLAGS_RETRACT
        # ardupilotmega.GIMBAL_MANAGER_FLAGS_NEUTRAL
        msg = ardupilotmega.MAVLink_gimbal_manager_set_pitchyaw_message(
            0,
            0,
            flags=32  ,
            gimbal_device_id=1,
            pitch=pitch if not is_rates else math.nan,
            yaw=yaw if not is_rates else math.nan,
            pitch_rate= pitch if is_rates else math.nan,
            yaw_rate= yaw if is_rates else math.nan
        )
        self.master.mav.send(msg)

    def run(self):
        try:
            t = self.tf_buffer.lookup_transform(CAMERA_LINK_FRAME, OBJECT_FRAME, rclpy.time.Time())
        except Exception as e:
            print("Cant find the transform")
            t = self.neutral_tf_creator()
        print(t)
        pid_out = self.yaw_pid(-t.transform.translation.y)
        self.manager_set_pitchyaw(0.0, pid_out, is_rates=True)
        # print(pid_out)

def main(args=None):
    rclpy.init(args=args)
    gimbal_control = GimbalControl()
    rclpy.spin(gimbal_control)
    gimbal_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()