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
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from scipy.spatial.transform import Rotation
from apm_application.consts import *


class ObjectTracker(Node):
    def __init__(self):
        super().__init__('object_tracker')
        self.get_logger().info("object tracker running")
        self.publisher_ = self.create_publisher(String, 'object_coordinates', 10)
        self.image_publisher_ = self.create_publisher(Image, 'object_tracking/image_raw', 10)
        self.subscription = self.create_subscription(Image, CAMERA_TOPIC, self.image_callback, 10)
        # self.odom_subscriber = self.create_subscription(Odometry, CAMERA_ODOM_TOPIC, lambda msg: self.odom_to_tf_broadcaster(msg, CAMERA_OPTICAL_FRAME), 10)
        # self.odom_subscriber = self.create_subscription(Odometry, CAMERA_PHYSICAL_ODOM_TOPIC,  lambda msg: self.odom_to_tf_broadcaster(msg, CAMERA_LINK_FRAME), 10)


        self.tf_broadcaster = TransformBroadcaster(self)
        self.bridge = CvBridge()

    def odom_to_tf_broadcaster(self, msg: Odometry, child_frame):
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = child_frame

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

    def send_tf_from_r_t(self, tvec, rvec, parent_frame = CAMERA_OPTICAL_FRAME, child_frame = OBJECT_FRAME):
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = float(tvec[0])
        t.transform.translation.y = float(tvec[1])
        t.transform.translation.z = float(tvec[2])
              # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        rmat, _ = cv2.Rodrigues(rvec)
        # convert the matrix to a quaternion
        q = Rotation.from_matrix(rmat).as_quat()

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        #print("transform published: ", t)
        # Send the transformation
        self.tf_broadcaster.sendTransform(t)

    def init_transforms(self):
        self.camera_in_world = TransformBroadcaster()
        self.object_in_camera_optical = TransformBroadcaster()


    def image_callback(self, msg):
        mask = 0
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # fg_mask = self.back_sub.apply(frame)
        # fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_CLOSE, self.kernel)
        # fg_mask = cv2.medianBlur(fg_mask, 5)
        # _, fg_mask = cv2.threshold(fg_mask, 127, 255, cv2.THRESH_BINARY)
        mask += cv2.inRange(hsv, (58, 25, 25), (70, 255,255))
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # cv2.imshow("debug", mask)
        # cv2.waitKey(1)
        areas = [cv2.contourArea(c) for c in contours]

        if len(areas) < 1:
            self.publish_image(frame)
            return

        max_index = np.argmax(areas)
        cnt = contours[max_index]
        x, y, w, h = cv2.boundingRect(cnt)

        object_size = 0.5
        obj_p = np.array([
                [-object_size, -object_size, -object_size],
                [object_size, -object_size, -object_size], 
                [object_size, object_size, -object_size], 
                [-object_size, object_size, -object_size]
            ],dtype=np.float32)
        
        img_p = np.array([
            (x, y),
            (x + w, y),
            (x + w, y + h),
            (x, y + h)
        ],dtype=np.float32)

        # # 3d points
        # img_p = np.zeros((6*9,3), np.float32)
        # img_p[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)

        #print(x,y,w,h)
        size = (640, 512)
        fl = 686
        camera_matrix = np.array(
            [[fl, 0, size[0]/2],
             [0, fl, size[1]/2],
             [0,0,1]], dtype=np.float32
        )

        dist_coeffs = np.zeros((4,1)) # no lens distortion
        # success, rotation_vector, translation_vector = cv2.solvePnPRansac(
        #     obj_p, img_p, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_RANSAC
        # )

        # if success:
        #     print("Rotation Vector:", rotation_vector)
        #     print("Translation Vector:", translation_vector)

        _, rvec, tvec= cv2.solvePnP(obj_p, img_p, camera_matrix, dist_coeffs)
        # print(tvec)
        # self.draw(frame,(int(x),int(y)),imgpts)
        cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 1.2)
        self.send_tf_from_r_t(tvec, rvec)
        # # Convert rotation vector to Euler angles
        # rmat, _ = cv2.Rodrigues(rvec)
        # pitch = math.atan2(rmat[2, 0], rmat[2, 2])
        # yaw = math.atan2(rmat[2, 1], rmat[2, 2])

        # # Calculate position
        # position = tvec[:2]  
        # print(tvec)
        cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 3)

        x2 = x + int(w/2)
        y2 = y + int(h/2)
        cv2.circle(frame, (x2, y2), 4, (255, 0, 0), -1)

        text = "x: " + str(x2) + ", y: " + str(y2)
        cv2.putText(frame, text, (x2 - 10, y2 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        self.publisher_.publish(String(data=text))

        self.publish_image(frame)

    def publish_image(self, frame):
        image_message = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.image_publisher_.publish(image_message)

def main(args=None):
    rclpy.init(args=args)
    object_tracker = ObjectTracker()
    rclpy.spin(object_tracker)
    object_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()