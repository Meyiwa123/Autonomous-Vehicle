import os
from launch_ros.actions import Node
from launch import LaunchDescription
from nav2_common.launch import RewrittenYaml
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    velodyne_driver_node = Node(
        package='velodyne_driver',
        executable='velodyne_driver_node',
        name='velodyne_driver_node',
        parameters=[{'model': 'VLP16'}],
    )
    velodyne_transform_node = Node(
        package='velodyne_pointcloud',
        executable='velodyne_transform_node',
        name='velodyne_transform_node',
        parameters=[{'model': 'VLP16'}],
    )
    velodyne_laserscan_node = Node(
        package='velodyne_laserscan',
        executable='velodyne_laserscan_node',
        name='velodyne_laserscan_node',
    )

    ublox_node = Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        name='ublox_gps_node',
        output='both',
        parameters=[os.path.join('config', 'neo_m8u_rover.yaml')]
    )

    robot_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join('launch', 'dual_ekf_navsat.launch.py'))
    )

    bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_params = os.path.join('config', "nav2_no_map_params.yaml")
    configured_params = RewrittenYaml(
        source_file=nav2_params, root_key="", param_rewrites="", convert_types=True
    )
    navigation2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "True",
            "params_file": configured_params,
            "autostart": "True",
        }.items(),
    )

    zed_ros2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('zed_wrapper'), 'launch', 'zed2.launch.py')
        )
    )

    gps_navigation_cmd = Node(
        package='av',
        executable='gps_navigation',
        name='gps_navigation',
        output='screen'
    )

    dashboard_cmd = Node(
        package='av',
        executable='dashboard',
        name='dashboard',
        output='screen'
    )

    display_lane_cmd = Node(
        package='av',
        executable='display_lane',
        name='display_lane',
        output='screen'
    )

    lane_steer_cmd = Node(
        package='av',
        executable='lane_steer',
        name='lane_steer',
        output='screen'
    )

    twist_mux_params = os.path.join('config', 'twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params],
    )

    flysky_cmd = Node(
        package='av',
        executable='flysky',
        name='flysky',
        output='screen'
    )

    return LaunchDescription([
        velodyne_driver_node,
        velodyne_transform_node,
        velodyne_laserscan_node,
        ublox_node,
        navigation2_cmd,
        robot_localization_cmd,
        zed_ros2_cmd,
        gps_navigation_cmd,
        dashboard_cmd,
        display_lane_cmd,
        lane_steer_cmd,
        twist_mux,
        flysky_cmd
    ])
