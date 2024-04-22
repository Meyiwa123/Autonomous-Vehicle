import os
from launch_ros.actions import Node
from launch import LaunchDescription
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

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='lifelong_launch.py',
        name='slam_toolbox',
        output='screen',
        parameters=[{'use_sim_time': False}]
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

    teleop_twist_keyboard_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        remappings=[('/cmd_vel', '/cmd_vel_keyboard')],
    )

    twist_mux_params = os.path.join('config', 'twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params],
    )

    # Get a list of all Python files in the source directory
    source_files = [f for f in os.listdir('src') if f.endswith('.py')]
    # Create a Node for each Python file
    nodes = [
        Node(
            package='aav',
            executable=f,
            name=f'{f[:-3]}_node',
            output='screen'
        )
        for f in source_files
    ]

    return LaunchDescription([
        velodyne_driver_node,
        velodyne_transform_node,
        velodyne_laserscan_node,
        ublox_node,
        #slam_toolbox_node,
        navigation2_cmd,
        teleop_twist_keyboard_node,
        robot_localization_cmd,
        twist_mux,
        *nodes
    ])
