#!/bin/bash

# Read topics from file
topics=$(cat /workspaces/apm_24_9/src/mav_drone_localization/config/topics_for_bag.txt | tr '\n' ' ')

now=$(date +"%Y_%m_%d_%H_%M_%S")

# Record all the topics
ros2 bag record -o /workspaces/apm_24_9/src/mav_drone_localization/rosbag/$now.bag $topics