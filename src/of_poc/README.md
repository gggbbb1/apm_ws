# Optical flow
## API
### inputs:
- ROS Image
- Camera Orientation + Height - Odometry msg.
    - Mendatory: pose.orientation: camera frame for orientation is FLU (z-up, x-front, y-left).
    - Mendatory: pose.position.z: height of the camera.
    - Optional: twist.linear.x/y: will be published as ground truth for velocities.
    - Optional: pose.position.x/y: if test_twist_vs_gazebo will run - it will publish the pose error between this values & the integration over velocities.
- Camera_Info

### outputs
- TwistWithCovarianceStamped
- TwistStamped

## Preperation
- in gazebo (to use fully on simulation): add the p3d plugin:
    ```xml
        <plugin name="pose_3d_plugin_link" filename="libgazebo_ros_p3d.so">
        <ros>
            <namespace>odom</namespace>
            <remapping>odom:=camera_flu</remapping>
            </ros>
            <body_name>fake_camera_link</body_name>
            <gaussian_noise>0.01</gaussian_noise>
            <frame_name>map</frame_name>
            <update_rate>200</update_rate>
            </plugin>
    ``` 

## Usage
- change topic names using the consts file or in your launch file.
- change parameters using the of_lucas.yaml
- run:
```bash
ros2 launch of_poc of_system.launch.py
# and
ros2 run of_poc test_twist_vs_gz
# and
ros2 launch of_poc gui_tools.launch.xml
```
## TODO:
- add covariances for inputs?