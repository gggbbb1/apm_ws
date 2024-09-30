import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import MultiThreadedExecutor
import numpy as np
from threading import Thread
import os 
import rclpy.time
import tf2_ros
from tf2_ros import TransformListener, tf2
from tf2_ros.buffer import Buffer
import tf2_geometry_msgs
from geometry_msgs.msg import Vector3Stamped, TwistWithCovarianceStamped

# Constants
MARKER_SIZE = 3
RESULT_PDF_PATH = '/workspaces/apm_24_9/src/mav_drone_localization/plots/check_z_error2.pdf'
SUBTITLES_SIZE = 20
BACKGROUND_POSE = (0.9,1,0.9)
BACKGROUND_VEL = (1,0.9,1)


class OdometryPlotter(Node):
    def __init__(self):
        super().__init__('odometry_plotter')

        # Subscribing to two odometry topics with SensorDataQoS
        self.odom_sub_1 = self.create_subscription(
            Odometry, '/odom1', self.odom1_callback, qos_profile_sensor_data)
        self.odom_sub_2 = self.create_subscription(
            Odometry, '/odom2', self.odom2_callback, qos_profile_sensor_data)
        self.twist_sub = self.create_subscription(
            TwistWithCovarianceStamped, '/twist', self.twist_callback, qos_profile_sensor_data)

        # Storing odometry data and timestamps
        self.odom1_data = []
        self.odom2_data = []
        self.twist_data = []

        self.declare_parameter('result_pdf_folder', '/workspaces/apm_24_9/src/mav_drone_localization/plots')
        self.declare_parameter('result_pdf_name', 'latest')
        self.declare_parameter('delay', 6.0)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.start_time = time.time()

        pdf_path = os.path.join(self.get_parameter('result_pdf_folder').value, self.get_parameter('result_pdf_name').value) + '.pdf'
        # Initialize PDF document for plots
        self.pdf_filename = pdf_path

    def odom1_callback(self, msg: Odometry):
        if time.time() - self.start_time > self.get_parameter('delay').value:
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            pos1 = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
            orientation1 = self.euler_from_quaternion(msg.pose.pose.orientation)
            # vel1 = (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z,
            #         msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z)
            # transform velocity from map to body:
            try:
                current_transform = self.tf_buffer.lookup_transform('map', msg.child_frame_id, rclpy.time.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().error(f"Failed to lookup transform: {e}")
                return
            lin_vel = Vector3Stamped()
            lin_vel.vector.x = msg.twist.twist.linear.x
            lin_vel.vector.y = msg.twist.twist.linear.y
            lin_vel.vector.z = msg.twist.twist.linear.z

            ang_vel = Vector3Stamped()
            ang_vel.vector.x = msg.twist.twist.angular.x
            ang_vel.vector.y = msg.twist.twist.angular.y
            ang_vel.vector.z = msg.twist.twist.angular.z
            vec_lin_trans = tf2_geometry_msgs.do_transform_vector3(lin_vel, current_transform)
            self.get_logger().error(f"{vec_lin_trans}")
            vec_ang_trans = tf2_geometry_msgs.do_transform_vector3(ang_vel, current_transform)
            vel1 = (vec_lin_trans.vector.x, vec_lin_trans.vector.y, vec_lin_trans.vector.z, vec_ang_trans.vector.x, vec_ang_trans.vector.y, vec_ang_trans.vector.z)
            self.odom1_data.append((timestamp, pos1, orientation1, vel1))

    def odom2_callback(self, msg: Odometry):
        if time.time() - self.start_time > self.get_parameter('delay').value:
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            pos2 = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
            orientation2 = self.euler_from_quaternion(msg.pose.pose.orientation)
            vel2 = (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z,
                    msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z)
            
            self.odom2_data.append((timestamp, pos2, orientation2, vel2))

    def twist_callback(self, msg: TwistWithCovarianceStamped):
        if time.time() - self.start_time > self.get_parameter('delay').value:
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            # try:
            #     current_transform = self.tf_buffer.lookup_transform('base_link', 'map', rclpy.time.Time())
            # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            #     self.get_logger().error(f"Failed to lookup transform: {e}")
            #     return
            # lin_vel = Vector3Stamped()
            # lin_vel.vector.x = msg.twist.twist.linear.x
            # lin_vel.vector.y = msg.twist.twist.linear.y
            # lin_vel.vector.z = msg.twist.twist.linear.z
            
            # vec_lin_trans = tf2_geometry_msgs.do_transform_vector3(lin_vel, current_transform)
            
            # vel = (vec_lin_trans.vector.x, vec_lin_trans.vector.y)
            vel = (msg.twist.twist.linear.x, msg.twist.twist.linear.y)
            self.twist_data.append((timestamp, vel))

    def euler_from_quaternion(self, orientation):
        import tf_transformations
        return tf_transformations.euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])

    def add_grid(self, ax):
        ax.grid(True, linestyle='--', alpha=0.5)

    def add_lines(self, ax, x, y, color):
        ax.plot(x, y, color, alpha=0.5)

    def save_plots(self):
        with PdfPages(self.pdf_filename) as pdf_pages:
            if len(self.odom1_data) > 0 and len(self.odom2_data) > 0:
                # Process the odometry data
                timestamps1, pos1, orientation1, vel1 = zip(*self.odom1_data)
                timestamps2, pos2, orientation2, vel2 = zip(*self.odom2_data)
                if len(self.twist_data) > 0:
                    timestamp_twist, twist = zip(*self.twist_data)
                    twist_np = np.array(twist)
                    time_twist_np = np.array(timestamp_twist)
                else:
                    twist_np = np.array([[],[]])
                    time_twist_np = np.array([])
                
                # Extract positions, orientations, and velocities
                pos1 = np.array(pos1)
                pos2 = np.array(pos2)
                orientation1 = np.array(orientation1)
                orientation2 = np.array(orientation2)
                vel1 = np.array(vel1)
                vel2 = np.array(vel2)

                
                # Plot Pose X-Y without time
                self.create_2_tracks_map(pdf_pages, pos1, pos2, 'Map in X-Y', BACKGROUND_POSE)

                # Plot Pose X vs. Time
                self.create_single_plot(pdf_pages, timestamps1, pos1[:, 0], timestamps2, pos2[:, 0], 'Pose X', 'dist (m)', BACKGROUND_POSE)

                 # Plot Pose Y vs. Time
                self.create_single_plot(pdf_pages, timestamps1, pos1[:, 1], timestamps2, pos2[:, 1], 'Pose Y', 'dist (m)', BACKGROUND_POSE)

                # Plot Pose Z vs. Time
                self.create_single_plot(pdf_pages, timestamps1, pos1[:, 2], timestamps2, pos2[:, 2], 'Pose Z', 'Alt (m)', BACKGROUND_POSE)


                # Plot Orientation vs. Time
                self.create_triple_plot(pdf_pages, timestamps1, orientation1, timestamps2, orientation2, 'RPY Orientation', BACKGROUND_POSE)

                # Plot Velocities X vs. Time
                self.create_single_plot(pdf_pages, timestamps1, vel1[:, 0], timestamps2, vel2[:, 0], 'Velocity X', 'Velocity (m/s)', BACKGROUND_VEL,twist_np[:,0],time_twist_np )


                # Plot Velocities Y vs. Time
                self.create_single_plot(pdf_pages, timestamps1, vel1[:, 1], timestamps2, vel2[:, 1], 'Velocity Y', 'Velocity (m/s)', BACKGROUND_VEL,twist_np[:,1],time_twist_np )


                # Plot Velocities Z vs. Time
                self.create_single_plot(pdf_pages, timestamps1, vel1[:, 2], timestamps2, vel2[:, 2], 'Velocity Z', 'Velocity (m/s)', BACKGROUND_VEL)

                # Plot Orientation Velocities Roll vs. Time
                self.create_triple_plot(pdf_pages, timestamps1, vel1[:, 3:6], timestamps2, vel2[:, 3:6], 'RPY Velocities', BACKGROUND_VEL)


    def create_2_tracks_map(self, pdf_pages, pos1, pos2, title='', bg_color='white'):
        fig = plt.figure()
        fig.set_facecolor(bg_color)
        plt.plot(pos1[:, 0], pos1[:, 1], 'bo', markersize=MARKER_SIZE, label='EKF Result')
        plt.plot(pos2[:, 0], pos2[:, 1], 'ro', markersize=MARKER_SIZE, label='Ground Truth')
        self.add_lines(plt.gca(), pos1[:, 0], pos1[:, 1], 'b--')
        self.add_lines(plt.gca(), pos2[:, 0], pos2[:, 1], 'r--')
        plt.title(title, size=SUBTITLES_SIZE)
        plt.xlabel('X Position')
        plt.ylabel('Y Position')
        plt.legend()
        ax = plt.gca()
        self.add_grid(ax)
        pdf_pages.savefig()
        plt.close()

    def create_single_plot(self, pdf_pages, timestamps1, vel1, timestamps2, vel2, title,  y_label, bg_color='white', more_twist=None, more_twist_stamp=None):
        fig = plt.figure()
        fig.set_facecolor(bg_color)
        plt.plot(timestamps1, vel1, 'bo', markersize=MARKER_SIZE, label='EKF Result')
        plt.plot(timestamps2, vel2, 'ro', markersize=MARKER_SIZE, label='Ground Truth')

        if more_twist is not None:
            plt.plot(more_twist_stamp, more_twist, 'yo', markersize=MARKER_SIZE, label='Twist Data')
            self.add_lines(plt.gca(), more_twist_stamp, more_twist, 'y--')


        self.add_lines(plt.gca(), timestamps1, vel1, 'b--')
        self.add_lines(plt.gca(), timestamps2, vel2, 'r--')

        plt.title(title, size=SUBTITLES_SIZE)
        plt.xlabel('Time (s)')
        plt.ylabel(y_label)
        plt.legend()
        ax = plt.gca()
        self.add_grid(ax)
        pdf_pages.savefig()
        plt.close()


    def create_triple_plot(self, pdf_pages, timestamps1, orientation1, timestamps2, orientation2, title='', bg_color='white'):
        fig, axes = plt.subplots(3, 1, figsize=(8, 12))  # Increased figsize height
        fig.suptitle(title, size=40)
        fig.set_facecolor(bg_color)

        axes[0].plot(timestamps1, orientation1[:, 0], 'bo', markersize=MARKER_SIZE, label='EKF Result Roll')
        axes[0].plot(timestamps2, orientation2[:, 0], 'ro', markersize=MARKER_SIZE, label='Ground Truth Roll')
        self.add_lines(axes[0], timestamps1, orientation1[:, 0], 'b--')
        self.add_lines(axes[0], timestamps2, orientation2[:, 0], 'r--')
        axes[0].set_title('Roll Comparison', size=SUBTITLES_SIZE)
        axes[0].set_xlabel('Time (s)')
        axes[0].set_ylabel('Roll')
        axes[0].legend()
        self.add_grid(axes[0])

        axes[1].plot(timestamps1, orientation1[:, 1], 'bo', markersize=MARKER_SIZE, label='EKF Result Pitch')
        axes[1].plot(timestamps2, orientation2[:, 1], 'ro', markersize=MARKER_SIZE, label='Ground Truth Pitch')
        self.add_lines(axes[1], timestamps1, orientation1[:, 1], 'b--')
        self.add_lines(axes[1], timestamps2, orientation2[:, 1], 'r--')
        axes[1].set_title('Pitch Comparison', size=SUBTITLES_SIZE)
        axes[1].set_xlabel('Time (s)')
        axes[1].set_ylabel('Pitch')
        axes[1].legend()
        self.add_grid(axes[1])

        axes[2].plot(timestamps1, orientation1[:, 2], 'bo', markersize=MARKER_SIZE, label='EKF Result Yaw')
        axes[2].plot(timestamps2, orientation2[:, 2], 'ro', markersize=MARKER_SIZE, label='Ground Truth Yaw')
        self.add_lines(axes[2], timestamps1, orientation1[:, 2], 'b--')
        self.add_lines(axes[2], timestamps2, orientation2[:, 2], 'r--')
        axes[2].set_title('Yaw Comparison', size=SUBTITLES_SIZE)
        axes[2].set_xlabel('Time (s)')
        axes[2].set_ylabel('Yaw')
        axes[2].legend()
        self.add_grid(axes[2])

                # Adjust layout to add space between subplots
        plt.subplots_adjust(hspace=0.4)  # Increase vertical space between plots

        pdf_pages.savefig(fig)
        plt.close(fig)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryPlotter()
    
    # Use MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        # Run the executor in a separate thread
        executor_thread = Thread(target=executor.spin)
        executor_thread.start()

        # Allow the node to run for a certain duration or until interrupted
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Save plots and shut down
        node.save_plots()

        node.destroy_node()
        rclpy.shutdown()
        executor_thread.join()

if __name__ == '__main__':
    main()
