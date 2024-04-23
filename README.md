# Autonomous Vehicle
This project aims to create an autonomous vehicle capable of executing critical tasks essential for safe and efficient autonomous driving. The system is implemented using the ROS2 framework.

## Requirements
1. Ensure [ROS Humble](https://docs.ros.org/en/humble/Installation.html) is installed and the workspace is sourced
    ```
    source /opt/ros/humble/setup.bash
    ```

2. Ensure that the following ROS2 packages are installed:
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

## Installation
1. Clone this repository into your ROS2 workspace
2. Install all python dependencies are installed
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

## Configuration 
1. Create, obtain and set a Open Route Service API key in `gps_nvaigation.py`.
2. Default port GPS is connected to is `/dev/ttyACM0`, look into `neo_m8u_rover.yaml` to change. 
3. Default port Flysky controller (arduino) is connected to `/dev/ttyUSB0` look into `flysky_controller.py` to change it.

## Set-up Velodyne Lidar
1. Configure your network settings to manual ip address of `192.168.1.201` and mask of `255.255.255.0`
2. If you are using a virtual machine
    * Within the network settings of the VM use a `Bridged Adapter` to your `ethernet card`
    * Set ipv4 address to `192.168.1.77` and subnet to `255.255.255.0`
2. Configure the LiDAR in your browse at `http://192.168.1.201/`

## Set-up Zed Ros Wrapper
To install the zed_ros2_wrapper, open a bash terminal, clone the package from Github, and build it:
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

## Launch Packgae
There are multiple launch packages, each with its specific purpose:
* The command `ros2 launch av gps_navigation.launch.py` orchestrates the setup and execution of nodes related to GPS-based navigation in an autonomous vehicle context. 
* On the other hand, `ros2 launch av teleop_carla_sim.launch.py` initiates a teleoperation setup for controlling a vehicle within the Carla simulation environment.

## Additional Resources 
1. [Ublox](https://github.com/KumarRobotics/ublox/tree/ros2)
2. [Velodyne](https://github.com/ros-drivers/velodyne)
3. [Slam Toolbox](https://github.com/SteveMacenski/slam_toolbox)
4. [Navigation 2](https://github.com/ros-planning/navigation2)
5. [Zed Ros2 Wrapper](https://github.com/stereolabs/zed-ros2-wrapper)
6. [Robot Localization](https://github.com/cra-ros-pkg/robot_localization)
7. [CAN-BUS Shield V2.0](https://wiki.seeedstudio.com/CAN-BUS_Shield_V2.0/)
8. [Converting a Point Cloud to a 3D Model](https://gazebosim.org/api/gazebo/4.0/pointcloud.html)
9. [Getting Started with the Velodyne VLP16](https://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16)
10. [Enabling CAN on Nvidia Jetson Xavier Developer Kit](https://medium.com/@ramin.nabati/enabling-can-on-nvidia-jetson-xavier-developer-kit-aaaa3c4d99c9)

## Note
1. If cloning the requried packages from Github and building them, you may encounter an error if your device runs out of memory (occasionally on virtual machines) because by default ROS uses a parallel build system to build packages concurrently. You should make use of `--parallel-workers` and/or `--executor sequential` to limit the parallel compilation.
2. When using the flysky controller it is required to be connected through an arduino to the Jetson. This is because it cannot be directly connected as the Tx and Rx pins are used by the CAN.