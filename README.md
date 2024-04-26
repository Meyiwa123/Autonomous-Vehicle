# Autonomous Vehicle
This project aims to create an autonomous vehicle capable of executing critical tasks essential for safe and efficient autonomous driving.

## Project Structure
The project is organized as follows. The `./docs` folder contains additional documentation and resources, and `./utils` has some additional scripts for auxillary functions. Under the `./docker` folder as different docker files to create each image:
* carla_client_node: Is a container configred to connect to a running carla container and interface with it, current it simulates Lidar and Camera data.
* foxglove_bridge: This is a container that builds a foxglove bridge.
* ros2_container: This is a container with ROS2 installed and configured in it, it runs the autonomous vehicle code. 

## Deployment using Docker
To deploy locally, simply run: `docker-compose up`. This will run in the current terminal so you will be able to see all the logs from each container displayed - you can shut everything down from here by hitting `Ctrl+C`. You can also run in detached mode which deploys all services in the background by running `docker-compose up -d ` then you can shut everything down by running `docker-compose down`.

**Note:** The first time you run this it may take a while since there are quite a few large images that need to be build and downloaded - ensure you have ~30GB of free space to be safe.


## Deployment Locally

### Requirements
1. It is recommended to run the application on a Linux device, additionally, the system will require a Nvidia GPU. It can be configured from [Nvidia Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#installation-guide.)
    ```
    # Check Nvidia Drivers Versions
    nvidia-smi
    # Check CUDA Driver
    /usr/local/cuda/bin/nvcc --version
    ``` 
2. Ensure [ROS Foxy](https://docs.ros.org/en/foxy/Installation.html) is installed and the workspace is sourced
    ```
    source /opt/ros/foxy/setup.bash
    ```
3. Ensure the following ROS2 packages are installed:
* ublox
* mapviz
* velodyne
* tile_map
* twist_mux
* mapviz_plugins
* navigation 2
* zed-ros2-wrapper
* robot_localization
* teleop-twist-keyboard
    The packages can be installed by running:
    ```
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
    ```
#### Set-up Zed Ros Wrapper
To install the [Zed Ros2 Wrapper](https://github.com/stereolabs/zed-ros2-wrapper), open a bash terminal, clone the package from Github, and build it:
```
mkdir -p ~/ros2_ws/src/ # create your workspace if it does not exist
cd ~/ros2_ws/src/ #use your current ros2 workspace folder
git clone  --recursive https://github.com/stereolabs/zed-ros2-wrapper.git
cd ..
sudo apt update
rosdep install --from-paths src --ignore-src -r -y # install dependencies
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc) # build the workspace
echo source $(pwd)/install/local_setup.bash >> ~/.bashrc # automatically source the installation in every new bash (optional)
source ~/.bashrc
```

#### Set-up Velodyne Lidar
1. Configure your network settings to manual ip address of `192.168.1.201` and mask of `255.255.255.0`
2. If you are using a virtual machine
    * Within the network settings of the VM use a `Bridged Adapter` to your `ethernet card`
    * Set ipv4 address to `192.168.1.77` and subnet to `255.255.255.0`
2. Configure the LiDAR in your browse at `http://192.168.1.201/`

## Configuuration
1. Create, obtain and set a Open Route Service API key in `gps_nvaigation.py`.
2. Default port GPS is connected to is `/dev/ttyACM0`, look into `neo_m8u_rover.yaml` to change. 
3. Default port Flysky controller (arduino) is connected to `/dev/ttyUSB0` look into `flysky_controller.py` to change it.

## Installation
1. Clone this repository into your ROS2 workspace 
2. Install all python dependencies from `requirements.txt` located in `docker/ros2_container`
    ```
    pip install -r requirements.txt
    ```
3. Build the ROS2 package
   ```
   cd /autonomous-vehicle
   colcon build --symlink-install
   ```
4. Source the package
    ```
    source install/local_setup.bash
    ```

## Launching the Packgae
There are multiple launch packages, each with its specific purpose:
* The command `ros2 launch av gps_navigation.launch.py` orchestrates the setup and execution of nodes related to GPS-based navigation in an autonomous vehicle context. 
* On the other hand, `ros2 launch av teleop_carla_sim.launch.py` initiates a teleoperation setup for controlling a vehicle within the Carla simulation environment.
* The file `map_server.launch.py` launched a map server for displaying the Carleton 
map, in which the data is stored in `map.pgm`.

## Additional Resources 
* [CAN-BUS Shield V2.0](https://wiki.seeedstudio.com/CAN-BUS_Shield_V2.0/)
* [Converting a Point Cloud to a 3D Model](https://gazebosim.org/api/gazebo/4.0/pointcloud.html)
* [Getting Started with the Velodyne VLP16](https://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16)
* [Enabling CAN on Nvidia Jetson Xavier Developer Kit](https://medium.com/@ramin.nabati/enabling-can-on-nvidia-jetson-xavier-developer-kit-aaaa3c4d99c9)

## Additional Note
1. If cloning the requried packages from Github and building them, you may encounter an error if your device runs out of memory (occasionally on virtual machines) because by default ROS uses a parallel build system to build packages concurrently. You should make use of `--parallel-workers` and/or `--executor sequential` to limit the parallel compilation.
2. When using the flysky controller it is required to be connected through an arduino to the Jetson. This is because it cannot be directly connected as the Tx and Rx pins are used by the CAN.
3. If scripts fail to run ensure they are executables with `chmod +x mypythonscript.py`