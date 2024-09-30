export PATH=$PATH:/home/user/.local/bin
sudo sysctl -w net.core.rmem_max=30000000
export CYCLONEDDS_URI=file://$PWD/cyclonedds.xml
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export PS1="🐳  \[\033[1;36m\]\h \[\033[1;34m\]\W\[\033[0;35m\] \[\033[1;36m\]# \[\033[0m\]"
export ROS_LOCALHOST_ONLY=1
source /usr/share/gazebo/setup.bash
source aliases.sh
source install/setup.bash