from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
   
    teleop_twist_keyboard_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        remappings=[('/cmd_vel', '/cmd_vel_keyboard')],
    )

    lane_steer_node = Node(
        package='av',
        executable='lane_steer',
        name='lane_steer',
        output='screen',
    )

    display_lane_node = Node(
        package='av',
        executable='display_lane',
        name='display_lane',
        output='screen',
    )

    carla_interface_ingress = Node(
        package='carla_interface',
        executable='carla_interface_ingress',
        name='carla_interface_ingress',
        output='screen',
    )

    carla_interface_egress = Node(
        package='carla_interface',
        executable='carla_interface_egress',
        name='carla_interface_egress',
        output='screen',
    )

    return LaunchDescription([
        teleop_twist_keyboard_node,
        lane_steer_node,
        display_lane_node,
        carla_interface_ingress,
        carla_interface_egress
    ])
