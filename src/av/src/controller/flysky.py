#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point32
import serial
import re

arduinoConnection = '/dev/ttyUSB0'


def myMap(inMin, inMax, outMin, outMax, x):
    return int((x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin)


class flysky_controller(Node):
    def __init__(self):
        super().__init__("flysky_controller")
        self.get_logger().info("Initializing flysky controller, please wait...")
        self.cmd_pub_ = self.create_publisher(Point32, "cmd_vel_flysky", 10)
        self.flysky_timer_ = self.create_timer(0.05, self.read_flysky_arduino)

        self.flysky = serial.Serial(arduinoConnection, 115200, timeout=1)
        self.flysky.reset_input_buffer()

        self.throttle_val_ = 0.0
        self.steering_val_ = 0.0
        self.braking_val_ = 0.0
        self.gear = 0
        self.throttle_last = 0
        self.steering_last = 0
        self.braking_last = 0

        self.get_logger().info("Flysky controller ready")

    def read_flysky_arduino(self):
        if self.flysky.in_waiting > 0:
            line = self.flysky.readline().decode('utf-8').rstrip()
            line = list((map(int, re.findall(r"-?\d+", line))))
            if (line != []):
                self.steering_val_ = line[0]
                if (self.steering_val_ < 3 and self.steering_val_ > -3):
                    self.steering_val_ = 0
                if (line[1] >= 0):
                    self.throttle_val_ = line[1]
                    self.braking_val_ = 0
                elif (line[1] < 15):
                    self.throttle_val_ = 0
                    self.braking_val_ = line[1]*-1
                else:
                    self.throttle_val_ = 0
                # line[3] not used currently
                # line[4] not used currently
                if (line[4] == 0):  # parking brake
                    self.throttle_val_ = 0
                    self.braking_val_ = 100
                self.gear = line[5]  # -1 is fwd, 1 is rev, 0 is neutral

                if (self.throttle_val_ < 3):
                    self.throttle_val_ = 0

                if (self.gear == 1):
                    self.throttle_val_ = self.throttle_val_*-1
                if (self.gear == 0):
                    self.throttle_val_ = 0

                self.send_cmd(Point32(x=self.throttle_val_,
                              y=self.steering_val_, z=self.braking_val_))
                
                self.flysky.reset_input_buffer()

    def send_cmd(self, cmd):
        self.cmd_pub_.publish(cmd)
        self.get_logger().info("Sending command: " + str(cmd))


def main(args=None):
    rclpy.init(args=args)

    node = flysky_controller()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print('\nController shutdown cleanly')


if __name__ == '__main__':
    main()
