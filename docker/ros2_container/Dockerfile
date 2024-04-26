# First stage sets up the base, ie everything except the active dev repo
FROM  osrf/ros:foxy-desktop-focal AS base

# Setup ROS 2 Workspace
SHELL ["/bin/bash", "-c"]

# Install required system dependencies
RUN sudo apt update && \
    sudo apt install -y \
        ros-foxy-ublox \
        ros-foxy-mapviz \
        ros-foxy-velodyne \
        ros-foxy-tile-map \
        ros-foxy-twist-mux \
        ros-foxy-mapviz-plugins \
        ros-foxy-navigation2 \
        ros-foxy-zed-ros2-wrapper \
        ros-foxy-robot-localization \
        ros-foxy-teleop-twist-keyboard \
        python3-colcon-common-extensions \
        git

# Clone Zed Ros Wrapper and build it
RUN mkdir -p /zed/src/ && \
    cd /zed/src/ && \
    git clone --recursive https://github.com/stereolabs/zed-ros2-wrapper.git && \
    cd .. && \
    sudo apt update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc) && \
    echo "source $(pwd)/install/local_setup.bash" >> ~/.bashrc

# Set-up Velodyne Lidar
RUN sudo ip addr add 192.168.1.77/24 dev eth0 && \
    sudo ip route add 192.168.1.201 dev eth0 && \
    echo "nameserver 8.8.8.8" | sudo tee /etc/resolv.conf >/dev/null

# RUN mkdir /ros2_ws
COPY ros2_ws ros2_ws 

# Install Python dependencies
COPY requirements.txt /
RUN pip install -r requirements.txt

RUN cd ros2_ws && \
    source /opt/ros/foxy/setup.bash && \
    colcon build

COPY ./ros_entrypoint.sh /

# Ensure script is executable
RUN ["chmod", "+x", "/ros_entrypoint.sh"]

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]