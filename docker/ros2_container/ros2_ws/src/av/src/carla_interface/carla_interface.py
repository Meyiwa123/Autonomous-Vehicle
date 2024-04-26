#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, NavSatFix
from std_msgs.msg import Float32
from carla_msgs.msg import CarlaEgoVehicleControl


class CarlaVehicleInterface(Node):
    def __init__(self):
        super().__init__('carla_vehicle_interface')

        # Subscribe to Carla's image and GPS data
        self.image_subscriber = self.create_subscription(
            Image, '/carla/ego_vehicle/camera/rgb/front/image_color', self.image_callback)
        self.gps_subscriber = self.create_subscription(
            NavSatFix, '/carla/ego_vehicle/gnss/gnss1/fix', self.gps_callback)
        # Subscribe to Carla's control data
        self.control_subscriber = self.create_subscription(
            CarlaEgoVehicleControl, '/carla/ego_vehicle/vehicle_control_cmd', self.control_callback)

        # Create publishers for throttle, brake, and steering topics
        self.throttle_publisher = self.create_publisher(
            Float32, 'throttle_topic', 10)
        self.brake_publisher = self.create_publisher(
            Float32, 'brake_topic', 10)
        self.steering_publisher = self.create_publisher(
            Float32, 'steering_topic', 10)

        # Create publishers for image and GPS data
        self.image_publisher = self.create_publisher(
            Image, 'image_topic', 10)
        self.gps_publisher = self.create_publisher(
            NavSatFix, 'gps_topic', 10)

    def image_callback(self, image_msg):
        # Process image data received from Carla
        self.image_publisher.publish(image_msg)

    def gps_callback(self, gps_msg):
        # Process GPS data received from Carla
        self.gps_publisher.publish(gps_msg)

    def control_callback(self, control_msg):
        # Process control message received from Nav2 or other source
        throttle = control_msg.throttle
        brake = control_msg.brake
        steering = control_msg.steer

        # Publish throttle, brake, and steering data
        throttle_msg = Float32()
        throttle_msg.data = throttle
        self.throttle_publisher.publish(throttle_msg)

        brake_msg = Float32()
        brake_msg.data = brake
        self.brake_publisher.publish(brake_msg)

        steering_msg = Float32()
        steering_msg.data = steering
        self.steering_publisher.publish(steering_msg)


if __name__ == '__main__':
    rclpy.init()
    vehicle_interface = CarlaVehicleInterface()
    rclpy.spin(vehicle_interface)
    rclpy.shutdown()
