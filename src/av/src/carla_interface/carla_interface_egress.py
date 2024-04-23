import math
import rclpy
from nav_msgs.msg import Path
from carla_msgs.msg import CarlaEgoVehicleControl


class Nav2ToCarlaEgress:
    def __init__(self):
        self.node = rclpy.create_node('nav2_to_carla_egress')

        # Subscribe to the navigation path topic from Navigation2
        self.nav_path_subscription = self.node.create_subscription(
            Path, '/nav2/path', self.nav_path_callback, 10)

        # Create publisher for sending control commands to Carla
        self.carla_control_publisher = self.node.create_publisher(
            CarlaEgoVehicleControl, '/carla/ego_vehicle/vehicle_control_cmd', 10)

    def nav_path_callback(self, path_msg):
        # Define maximum speed and maximum brake speed for Carla
        MAX_SPEED_KMH = 50  # km/h
        MAX_BRAKE_SPEED_KMH = 10  # km/h

        # Extract the last pose from the received path
        last_pose = path_msg.poses[-1]

        # Extract position and orientation information from the last pose
        position = last_pose.pose.position
        orientation = last_pose.pose.orientation

        # Calculate the steering angle from the orientation quaternion
        # Convert quaternion to Euler angles (roll, pitch, yaw)
        roll, pitch, yaw = self.euler_from_quaternion(
            orientation.x, orientation.y, orientation.z, orientation.w)

        # Calculate the steering angle from the yaw angle
        # In this example, we assume that the vehicle is facing in the x-axis direction (yaw = 0)
        # Adjust this calculation based on your vehicle's orientation and desired behavior
        steering_angle = math.degrees(yaw)

        # Generate control command for Carla
        carla_control_msg = CarlaEgoVehicleControl()

        # Set the calculated steering angle
        carla_control_msg.steer = steering_angle

        # Extract velocity information from the last pose
        linear_velocity = last_pose.twist.twist.linear.x

        # Convert linear velocity from m/s to km/h (Carla expects km/h)
        linear_velocity_kmh = linear_velocity * 3.6

        # Set throttle and brake based on the linear velocity
        if linear_velocity_kmh > 0:
            # Moving forward
            carla_control_msg.throttle = min(
                linear_velocity_kmh / MAX_SPEED_KMH, 1.0)
            carla_control_msg.brake = 0.0
        else:
            # Stopping or moving backward
            carla_control_msg.throttle = 0.0
            carla_control_msg.brake = min(-linear_velocity_kmh /
                                        MAX_BRAKE_SPEED_KMH, 1.0)

            # Publish control command to Carla
        self.carla_control_publisher.publish(carla_control_msg)

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert quaternion to Euler angles (roll, pitch, yaw).
        """
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use ±π/2 if out of range
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

def main():
    rclpy.init()
    nav2_to_carla_egress = Nav2ToCarlaEgress()
    rclpy.spin(nav2_to_carla_egress)
    nav2_to_carla_egress.node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
