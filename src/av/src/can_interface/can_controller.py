#!/usr/bin/env python3
import can
import time
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Point32
from teleop_twist_keyboard.msg import TwistStamped

class CANController(Node):
    def __init__(self):
        super().__init__('can_controller')

        # Initialize CAN bus interface
        self.can_bus_interface = can.interface.Bus(
            bustype='socketcan', channel='can0')

        # Subscribe to velocity commands    
        self.subscription_teleop = self.create_subscription(
            TwistStamped, 'cmd_vel_keyboard', self.twist_callback, 10)
        self.subscription_nav = self.create_subscription(
            Twist, 'cmd_vel', self.twist_callback, 10)
        self.subscription_flysky = self.create_subscription(
            Point32, 'cmd_vel_flysky', self.flysky_callback, 10)
        self.subscription = self.create_subscription(
            Float32, 'lane_steering_angle', self.lane_angle_callback, 10)
        self.radar_brake = self.create_subscription(
            Float32, 'radar_emergency_brake', self.radar_callback, 10)
        self.detection_brake = self.create_subscription(
            Float32, 'object_detection_brake', self.detection_callback, 10)
        
        self.stop = self.create_timer(0.01, self.stop_callback)

        # Define vehicle parameters
        self.wheelbase = 2.7
        self.brake_threshold_velocity = 0.1
        self.max_brake_percentage = 100

    def twist_callback(self, msg):
        linear_vel_x = msg.data.linear.x
        angular_vel_z = msg.data.angular.z
        # Convert velocity commands to throttle, steering, and brake commands
        throttle_command = linear_vel_x
        steering_command = math.atan(
            self.wheelbase * angular_vel_z / linear_vel_x)
        brake_percentage = min(1.0, abs(
            linear_vel_x) / self.brake_threshold_velocity) * self.max_brake_percentage
        self.send_can_message(0x100, throttle_command)
        self.send_can_message(0x101, steering_command)
        self.send_can_message(0x102, brake_percentage)

    def flysky_callback(self, msg):
        linear_vel_x = msg.data.x
        angular_vel_z = msg.data.y
        flysky_break = msg.data.z
        # Convert velocity commands to throttle, steering, and brake commands
        throttle_command = linear_vel_x
        # Convert angular velocity to angle (degrees)
        steering_command = math.degrees(angular_vel_z)
        self.send_can_message(0x100, throttle_command)
        self.send_can_message(0x101, steering_command)
        self.send_can_message(0x102, flysky_break)

    def lane_angle_callback(self, msg):
        steering_angle = msg.data
        self.send_can_message(0x101, steering_angle)

    def detection_callback(self, msg):
        brake_percentage = msg.data
        # Start from 1/4 brake percentage and increase gradually over one second
        for i in range(1, 101, 25):  # Start from 25% and increase by 25% each iteration
            self.send_can_message(0x102, brake_percentage * i / 100)
            time.sleep(0.25)  # Each segment lasts for 0.25 seconds
        # Stop for three seconds
        time.sleep(3)

    def radar_callback(self, msg):
        brake_percentage = msg.data
        self.send_can_message(0x102, brake_percentage)

    def stop_callback(self):
        # Send command to stop the vehicle
        throttle_command = 0
        steering_command = 0
        brake_percentage = 0
        self.send_can_message(0x100, throttle_command)
        self.send_can_message(0x101, steering_command)
        self.send_can_message(0x102, brake_percentage)

    def send_can_message(self, can_id, data):
        msg = can.Message(arbitration_id=can_id, data=[data])
        try:
            self.can_bus_interface.send(msg)
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
