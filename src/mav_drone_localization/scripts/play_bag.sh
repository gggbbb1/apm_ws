#!/bin/bash

# Request the user to input the bag file name
read -p "Enter the name of the bag file (with extension .bag): " bag_file

rosbag_path="/workspaces/apm_24_9/src/mav_drone_localization/rosbag"

# Check if the file exists
if [ ! -d "$rosbag_path/$bag_file" ]; then
  echo "Bag file '$rosbag_path/$bag_file' does not exist."
  exit 1
fi

# Request the user to input a ROS 2 domain ID
read -p "Enter a ROS 2 domain ID (e.g., 10): " domain_id

# Set the ROS 2 domain ID
echo "ROS_DOMAIN_ID set to $ROS_DOMAIN_ID"

# Play the bag file
ROS_DOMAIN_ID=$domain_id ros2 bag play "$rosbag_path/$bag_file"

# Reset the ROS 2 domain ID to its default value (optional)
unset ROS_DOMAIN_ID
