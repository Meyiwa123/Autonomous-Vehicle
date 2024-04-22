# AAV
This repository contains a ROS2 package for an autonomous vehicle project, demonstrating GPS waypoint following using Nav2 and forwarding vehicle commands using CAN communication.

## Requirements
1. Ensure ROS workspace is sourced
```
source /opt/ros/humble/setup.bash
```

2. Ensure all python dependencies are installed
```
pip install -r requirements.txt
```

3. Ensure that the following ROS2 packages are installed:
* navigation 2
* robot_localization
* teleop-twist-keyboard
* velodyne
* ublox -- refer to additional resources
* mapviz
* mapviz_plugins
* tile_map
* twist_mux

## Installation
1. Clone this repository into your ROS2 workspace
2. Build the ROS2 package
   ```
   cd /2023-2024
   colcon build --symlink-install
   ```
3. Source the package
    ```
    source install/local_setup.bash
    ```

## Configuration 
1. Create, obtain and set a Open Route Service API key in `gps_nvaigation.py`.
2. Default port GPS is connected to is `/dev/ttyACM0`, look into `neo_m8u_rover.yaml` to change. 
3. Default port Flysky controller (arduino) is connected to `/dev/ttyUSB0` look into `flysky_controller.py` to change it.

## Set-up Velodyne
1. Configure your network settings to manual ip address of `192.168.1.201` and mask of `255.255.255.0`
2. If you are using a virtual machine
    * Within the network settings of the VM use a `Bridged Adapter` to your `ethernet card`
    * Set ipv4 address to `192.168.1.77` and subnet to `255.255.255.0`
2. Configure the LiDAR in your browse at `http://192.168.1.201/`

## Launch Packgae
```
ros2 launch aav aav_bringup.launch.py
```

## Additional Resources 
1. [Slam Toolbox](https://github.com/SteveMacenski/slam_toolbox)
2. [Navigation 2](https://github.com/ros-planning/navigation2)
3. [Robot Localization](https://github.com/cra-ros-pkg/robot_localization)
4. [Ublox](https://github.com/KumarRobotics/ublox/tree/ros2)
5. [Velodyne](https://github.com/ros-drivers/velodyne)
6. [Getting Started with the Velodyne VLP16](https://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16)
7. [CAN-BUS Shield V2.0](https://wiki.seeedstudio.com/CAN-BUS_Shield_V2.0/)
8. [Enabling CAN on Nvidia Jetson Xavier Developer Kit](https://medium.com/@ramin.nabati/enabling-can-on-nvidia-jetson-xavier-developer-kit-aaaa3c4d99c9)
9. [Converting a Point Cloud to a 3D Model](https://gazebosim.org/api/gazebo/4.0/pointcloud.html)

## Note
1. If cloning the requried packages from Github and building them, you may encounter an error if your device runs out of memory (occasionally on virtual machines) because by default ROS uses a parallel build system to build packages concurrently. You should make use of `--parallel-workers` and/or `--executor sequential` to limit the parallel compilation.
2. When using the flysky controller it is required to be connected through an arduino to the Jetson. This is because it cannot be directly connected as the Tx and Rx pins are used by the CAN.