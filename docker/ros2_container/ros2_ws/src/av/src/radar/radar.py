#!/usr/bin/env python3
import can
import rclpy
import cantools
from rclpy.node import Node
from std_msgs.msg import Twist

class Radar(Node):
    def __init__(self):
        super().__init__("radar")

        self.radar = {}
        self.brake_pub_ = self.create_publisher(
            Twist, "radar_emergency_brake", 10)
        self.radar_timer_ = self.create_timer(0.01, self.radar_brake)

    def radar_brake(self):
        msg = Twist()  # Change this to your topic's message type
        db = cantools.database.load_file('ESR.dbc')   # load DBC file
        can_bus = can.interface.Bus(
            'can1', bustype='socketcan') 

        for i in range(1280, 1344):
            message = can_bus.recv()
            self.radar[f'Target{i-1279}'] = db.decode_message(i, message.data)

        # send brake signal if an object is less than 3m away
        for target_id, target_data in self.radar.items():
            if 0 < target_data["CAN_TX_TRACK_RANGE"] < 3.0:
                msg.linear.x = 0
                msg.angular.z = 0
                self.brake_pub_.publish(msg)
                break

def main():
    rclpy.init()
    node = Radar()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
