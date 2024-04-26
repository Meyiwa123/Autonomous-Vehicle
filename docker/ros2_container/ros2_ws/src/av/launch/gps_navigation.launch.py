import os
from launch_ros.actions import Node
from launch import LaunchDescription
from nav2_common.launch import RewrittenYaml
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    av_dir = get_package_share_directory("av")
    launch_dir = os.path.join(av_dir, 'launch')
    params_dir = os.path.join(av_dir, "config")
    nav2_params = os.path.join(params_dir, "nav2_no_map_params.yaml")
    configured_params = RewrittenYaml(
        source_file=nav2_params, root_key="", param_rewrites="", convert_types=True
    )

    velodyne_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'velodyne_launch.py')
        )
    )

    zed_ros2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(
                'zed_wrapper'), 'launch', 'zed2.launch.py')
        )
    )

    ublox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'ublox_gps_node-launch.py')
        )
    )

    robot_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'dual_ekf_navsat.launch.py'))
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

    twist_mux_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'twist_mux.launch.py')
        )
    )

    gps_navigation_cmd = Node(
        package='av',
        executable='gps_navigation.py',
        name='gps_navigation',
    )

    dashboard_cmd = Node(
        package='av',
        executable='dashboard.py',
        name='dashboard',
    )

    display_lane_cmd = Node(
        package='av',
        executable='display_lane.py',
        name='display_lane',
    )

    lane_steer_cmd = Node(
        package='av',
        executable='lane_steer.py',
        name='lane_steer',
    )

    flysky_cmd = Node(
        package='av',
        executable='flysky.py',
        name='flysky',
    )

    return LaunchDescription([
        velodyne_launch,
        ublox_launch,
        navigation2_cmd,
        robot_localization_cmd,
        zed_ros2_cmd,
        twist_mux_node,
        gps_navigation_cmd,
        dashboard_cmd,
        display_lane_cmd,
        lane_steer_cmd,
        flysky_cmd
    ])
