# lidarslam_ros2
This is a ROS2 package for LiDAR-based Simultaneous Localization and Mapping (SLAM). It provides functionalities for integrating LiDAR data to create maps and localize within them.
The usage instructions are of follows:

```
# Clone:
cd ~/ros2_ws/src
git clone --recursive https://github.com/rsasaki0109/lidarslam_ros2
cd ..
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Launch Velodyne
ros2 launch velodyne_driver velodyne_driver_node-VLP16-launch.py
ros2 launch velodyne_pointcloud  velodyne_transform_node-VLP16-launch.py

# Launch SLAM:
ros2 launch lidarslam lidarslam.launch.py
# OR
ros2 launch lidarslam lidarslam_tukuba.launch.py

# Launch SLAM RViZ
rviz2 -d src/lidarslam_ros2/lidarslam/rviz/mapping.rvix

# Launch Veldodyne RViz
ros2 run rviz rviz -f velodyne

# Save Map
ros2 service call /map_save std_srvs/Empty
```