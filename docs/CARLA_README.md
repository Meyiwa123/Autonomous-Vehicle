# Carla

## Requirements
* [foxy](https://docs.ros.org/en/foxy/Installation/Alternatives/Ubuntu-Development-Setup.html)
* Docker
* NVIDIA Container Toolkit

## Install Docker Engine
1. Uninstall conficting packages:
    ```
    for pkg in docker.io docker-doc docker-compose podman-docker containerd runc; do sudo apt-get remove $pkg; done
    ```
    
2. Set up Docker's apt repository.
    ```
    # Add Docker's official GPG key:
    sudo apt-get update
    sudo apt-get install ca-certificates curl
    sudo install -m 0755 -d /etc/apt/keyrings
    sudo curl -fsSL https://download.docker.com/linux/debian/gpg -o /etc/apt/keyrings/docker.asc
    sudo chmod a+r /etc/apt/keyrings/docker.asc
    # Add the repository to Apt sources:
    echo \
      "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/debian \
      $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
      sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
    sudo apt-get update
    ```
 
3. Install the Docker packages.
    ```
    sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
    ```
    
5. Add user to Docker Group
    ```
    sudo usermod -aG docker $USER
    ```

6. Restart docker services
    ```
    sudo systemctl restart docker
    ```
7. Verify that the installation is successful by running the hello-world image:
    ```
    sudo docker run hello-world
    ```

## Installing the NVIDIA Container Toolkit

1. Configure the production repository:
    ```
    curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
      && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
        sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
        sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
    ```
2. Update the packages list from the repository:
    ```
    sudo apt-get update
    ```
3. Install the NVIDIA Container Toolkit packages:
    ```
    sudo apt-get install -y nvidia-container-toolkit
    ```
    
## CARLA Debian Installation
1. Set up the Debian repository in the system:
    ```
    sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 1AF1527DE64CB8D9
    sudo add-apt-repository "deb [arch=amd64] http://dist.carla.org/carla $(lsb_release -sc) main"
    ```
2. Install CARLA and check for the installation in the /opt/ folder:
    ```
    sudo apt-get update # Update the Debian package index
    sudo apt-get install carla-simulator # Install the latest CARLA version, or update the current installation
    cd /opt/carla-simulator # Open the folder where CARLA is installed
    ```
This repository contains CARLA 0.9.10 and later versions. To install a specific version add the version tag to the installation command:
    ```
    apt-cache madison carla-simulator # List the available versions of Carla
    sudo apt-get install carla-simulator=0.9.10-1 # In this case, "0.9.10" refers to a CARLA version, and "1
    ```

## Running CARLA in a container
1. Pull the CARLA image `Take note of different versions`.
    ```
    # Pull a specific version
    docker pull carlasim/carla:0.9.12
    
    # Pull the latest image
    docker pull carlasim/carla:latest
    ```
2. Run the CARLA container with display `Take note of different versions`.
    ```
    #CARLA 0.9.12
    sudo docker run --privileged --gpus all --net=host -e DISPLAY=$DISPLAY carlasim/carla:0.9.12 /bin/bash ./CarlaUE4.sh
    
    # CARLA 0.9.12
    sudo docker run --privileged --gpus all --net=host -e DISPLAY=$DISPLAY -e SDL_VIDEODRIVER=x11 -v /tmp/.X11-unix:/tmp/.X11-unix:rw carlasim/carla:0.9.11 /bin/bash ./CarlaUE4.sh -vulkan <-additonal-carla-flags>
    ```
    
## ROS Bridge
1. Set up the project directory and clone the ROS bridge repository and submodules:
    ```
    mkdir -p ~/carla-ros-bridge && cd ~/carla-ros-bridge
    git clone --recurse-submodules https://github.com/carla-simulator/ros-bridge.git src/ros-bridge
    ```
2. Set up the ROS environment:
    ```
    source /opt/ros/foxy/setup.bash
    ```
3. Install the ROS dependencies:
    ```
    rosdep update
    rosdep install --from-paths src --ignore-src -r
    ```
4. Build the ROS bridge workspace using colcon:
    ```
    colcon build
    ```
    
## Run the ROS bridge


