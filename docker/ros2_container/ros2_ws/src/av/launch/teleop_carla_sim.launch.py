from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
   
    teleop_twist_keyboard_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen'    
    )

    lane_steer_node = Node(
        package='av',
        executable='lane_steer',
        name='lane_steer',
        output='screen',
    )

    display_lane_node = Node(
        package='av',
        executable='display_lane.py',
        name='display_lane',
        output='screen',
    )

    carla_interface = Node(
        package='carla_interface',
        executable='carla_interface',
        name='carla_interface',
        output='screen',
    )

    return LaunchDescription([
        teleop_twist_keyboard_node,
        lane_steer_node,
        display_lane_node,
        carla_interface
    ])
