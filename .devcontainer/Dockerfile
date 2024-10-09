FROM humble:dev

# ** [Optional] Uncomment this section to install additional packages. **
#
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
   && apt-get -y install --no-install-recommends \
      # # remove after build humble:dev again
      # ros-humble-gazebo-ros-pkgs \
      # python package
      python3-wxgtk4.0 \
      # ros packages
      ros-humble-cv-bridge \
      ros-humble-rosbridge-suite \
      ros-humble-tf-transformations \
      # for gui
      ros-humble-rviz2 \
      ros-humble-rqt \
      ros-humble-rqt-common-plugins \
      ros-humble-rqt-tf-tree \
      ros-humble-robot-localization \
      ros-humble-mavros \
      ros-humble-mavros-extras \
      ros-humble-rqt-runtime-monitor \
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=dialog

RUN /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh 

COPY .devcontainer/requirements.txt requirements.txt
RUN pip3 install -r requirements.txt