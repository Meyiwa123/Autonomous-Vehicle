import can
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Point32
from teleop_twist_keyboard.msg import TwistStamped

class CANController(Node):
    def __init__(self):
        super().__init__('can_controller')
        self.subscription_teleop = self.create_subscription(
            TwistStamped,
            'cmd_vel_keyboard',
            self.teleop_callback,
            10)
        self.subscription_nav = self.create_subscription(
            Twist,
            'cmd_vel',
            self.nav_callback,
            10)
        self.subscription_flysky = self.create_subscription(
            Point32,
            'cmd_vel_flysky',
            self.flysky_callback,
            10)
        self.subscription = self.create_subscription(
            Float32,
            'lane_steering_angle',
            self.lane_angle_callback,
            10)
        self.can_bus_throttle = can.interface.Bus(
            bustype='socketcan', channel='can0')
        self.can_bus_steering = can.interface.Bus(
            bustype='socketcan', channel='can1')
        self.can_bus_brake = can.interface.Bus(
            bustype='socketcan', channel='can2')

        # Define brake parameters
        self.brake_threshold_velocity = 0.1  # Adjust as needed
        self.max_brake_percentage = 100  # Adjust as needed

        # Variables to store the latest teleop, nav, and flysky commands
        self.latest_teleop_cmd = None
        self.latest_nav_cmd = None
        self.latest_flysky_cmd = None

        # Timer to send CAN messages
        self.timer = self.create_timer(0.05, self.cmd_vel_callback)

    def lane_angle_callback(self, msg):
        # Store the latest lane angle
        self.latest_lane_angle = msg.data

    def teleop_callback(self, msg):
        # Store the latest teleop command
        self.latest_teleop_cmd = msg.twist

    def nav_callback(self, msg):
        # Store the latest nav command
        self.latest_nav_cmd = msg

    def flysky_callback(self, msg):
        # Store the latest flysky command
        self.latest_flysky_cmd = msg

    def cmd_vel_callback(self):
        # Check if lane angle is available
        if hasattr(self, 'latest_lane_angle'):
            # Use lane angle for steering
            steering_angle = self.latest_lane_angle
            self.send_can_message(self.can_bus_steering, 0x101, steering_angle)
            self.latest_lane_angle = None  # Reset after use

        # Check if teleop command is available and use it with priority
        if self.latest_nav_cmd is not None:
            linear_vel_x = self.latest_nav_cmd.linear.x
            angular_vel_z = self.latest_nav_cmd.angular.z
            self.latest_nav_cmd = None  # Reset after use
        elif self.latest_teleop_cmd is not None:
            linear_vel_x = self.latest_teleop_cmd.linear.x
            angular_vel_z = self.latest_teleop_cmd.angular.z
            self.latest_teleop_cmd = None  # Reset after use
        elif self.latest_flysky_cmd is not None:
            linear_vel_x = self.latest_flysky_cmd.x
            angular_vel_z = self.latest_flysky_cmd.y
            flysky_break = self.latest_flysky_cmd.z
            self.latest_flysky_cmd = None  # Reset after use
        else:
            # If no commands are available, stop the vehicle
            linear_vel_x = 0.0
            angular_vel_z = 0.0

        # Convert velocity commands to throttle, steering, and brake commands (for demonstration purposes)
        throttle_command = linear_vel_x  # Assume linear velocity as throttle command
        # Convert angular velocity to angle (degrees)
        steering_command = math.degrees(angular_vel_z)

        # Calculate brake command based on the current velocity
        if linear_vel_x < self.brake_threshold_velocity:
            brake_percentage = min(1.0, abs(
                linear_vel_x) / self.brake_threshold_velocity) * self.max_brake_percentage
        else:
            brake_percentage = 0  # No brake applied

        # Send commands via CAN
        self.send_can_message(self.can_bus_throttle, 0x100, throttle_command)
        self.send_can_message(self.can_bus_steering, 0x101, steering_command)
        # If flysky brake is available, use it
        if flysky_break is not None:
            self.send_can_message(self.can_bus_brake, 0x102, flysky_break)
        else:
            self.send_can_message(self.can_bus_brake, 0x102, brake_percentage)

    def send_can_message(self, bus, can_id, data):
        msg = can.Message(arbitration_id=can_id, data=[data])
        try:
            bus.send(msg)
        except can.CanError:
            self.get_logger().error("Failed to send CAN message")


def main(args=None):
    rclpy.init(args=args)
    can_controller = CANController()
    rclpy.spin(can_controller)
    can_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
