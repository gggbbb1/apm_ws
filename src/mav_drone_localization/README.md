# robot_localization & MAVROS
## Parameters:
There are many.
### ekf.yaml (input for ekf_localization_node)
* ```smoothed_lagged_data``` - if i have sensor that is lagged, and will come later with a timestamp from before. (like: high CPU process)
* ```history_length``` - buffer length of the previous settings
* ```base_link_output_frame``` - what frame the output should be in.
### apm_config
* something cool = **target_position** ----> enable tf.listen between baselink and target position.
    same as target_attitude
* **global&local position** - need to change maybe the tf_frames (from map to odom maybe, some of it)

## NavSat Transform:
* datum -  the local origin in terms of global coordinates. scheme: ```[lat, lon, hdg (rad, 0=east), world_frame, body_frame]```
    that means: datum = EKF_ORIGIN. need to request set_msg_interval(49)
* exposes services about ```FromLL and ToLL``` - takes lat lon to map and vice versa.

## important points:
- the sensor msgs ariving **cant be with the same timestamp!!!!!**. the delta between msgs is a divider and therefore all of the data explodes.
- to get higher stream of mavlink msgs, the QGC/MP is fighting against you. need to enter application settings and set - ```[V] all values are set from vehicle params``` - set parameters: SR0_*
- the ```reject...``` parameters are defalut to ```double::max()``` if unspecified
- in the risetime of the ardupilot, the temperature builds until its stable. ~30 sec.
- timesync ardupilot-mavros: [https://ardupilot.org/dev/docs/ros-timesync.html](https://ardupilot.org/dev/docs/ros-timesync.html) - sys_time plugin.
- if IMU message comes without orientation ---> the first covariance value will be ```-1```. (for example, imu/data_raw)
- the ekf node ignores message if the timestamp equals the last one.
- GPS integration: ***FOR SIMULATION*** the covariance of the ```/mavros/global_position/fix/raw``` is determined in the SIM_GPS_ACC parameter. 
- ```differential & relative``` are connected only for pose data! 
- about mavros in general: most of the global points are transfered to the egm96 geoid model (which is more complicated than the WGS84 ellipsoid model). therefor, one should notice if the altitude data I recieve is according to which model. there is an option in ython to remove this values using the pgm file.

[https://help.propelleraero.com/hc/en-us/articles/19383617598743-What-Are-Ellipsoid-and-Geoid-Heights#:~:text=An%20ellipsoid%20is%20a%20mathematical,and%20undulation%20of%20the%20surface.](https://help.propelleraero.com/hc/en-us/articles/19383617598743-What-Are-Ellipsoid-and-Geoid-Heights#:~:text=An%20ellipsoid%20is%20a%20mathematical,and%20undulation%20of%20the%20surface.)

Example to work with it in python:
```python
#!/usr/bin/env python3
# Example code that helps you convert between AMSL and ellipsoid height
# To run this code you need:
#
# 1) the egm96-5.pgm file from geographiclib.
# To get it on Ubuntu run:
# sudo apt install geographiclib-tools
# sudo geographiclib-get-geoids egm96-5
#
# 2) PyGeodesy
# To get it using pip:
# pip install PyGeodesy

from pygeodesy.geoids import GeoidPGM

_egm96 = GeoidPGM('/usr/share/GeographicLib/geoids/egm96-5.pgm', kind=-3)

def geoid_height(lat, lon):
    """Calculates AMSL to ellipsoid conversion offset.
    Uses EGM96 data with 5' grid and cubic interpolation.
    The value returned can help you convert from meters 
    above mean sea level (AMSL) to meters above
    the WGS84 ellipsoid.

    If you want to go from AMSL to ellipsoid height, add the value.

    To go from ellipsoid height to AMSL, subtract this value.
    """
    return _egm96.height(lat, lon)
```
## ROSBAG
- there are 2 scripts in the package:
1. record_bag.sh - takes the topic list from ```pkg/config``` and records a bag file to ```pkg/rosbag``` directory. the name is the current time.
2. play_bag.sh - takes as argument the bag name & ros domain id and plays the correct bag file.

## Messages:
| msg name                        | msg type    | frame_id                                    | (child_frame_id) | robot_localization node (subscriber to this msg) |
|---------------------------------|-------------|---------------------------------------------|------------------|--------------------------------------------------|
| /mavros/imu_data                | IMU         | base_link    (in apm.config/imu)         | X                | odom & map & navsat                              |
| /rome_ekf_wrapper/alt_and_hdg | PoseWithCov | map  (in rome_ekf_wrapper/hard coded)                                   | X                | odom     & map                                         |
| /mavros/global_position/raw/fix | NavSatFix   | base_link. should be "frame of the antenna" (in apm.config/global_position) | X                | navsat                                           |
| /optical_flow/twist_with_cov | TwistWithCovStamped   | odom (in of_poc/consts/WORLD_FRAME) | X                | odom & map         |