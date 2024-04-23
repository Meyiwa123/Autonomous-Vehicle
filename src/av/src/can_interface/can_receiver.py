import can
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class CanReceiver(Node):
    def __init__(self):
        super().__init__('can_receiver')
        self.throttle_publisher = self.create_publisher(Float32, 'throttle_topic', 10)
        self.brake_publisher = self.create_publisher(Float32, 'brake_topic', 10)
        self.steering_publisher = self.create_publisher(Float32, 'steering_topic', 10)

        # Initialize CAN interface
        self.bus = can.interface.Bus(channel='can0', bustype='socketcan_native')

    def receive_can_messages(self):
        while rclpy.ok():
            message = self.bus.recv()
            if message.arbitration_id == 0x100:  # Throttle message ID
                throttle = message.data[0] / 255.0 * 100  # Assuming throttle value is in range [0, 255]
                self.publish_throttle(throttle)
            elif message.arbitration_id == 200:  # Brake message ID
                brake = message.data[0] / 255.0 * 100  # Assuming brake value is in range [0, 255]
                self.publish_brake(brake)
            elif message.arbitration_id == 0x300:  # Steering message ID
                steering = message.data[0]  # Assuming steering value is in range [0, 255]
                self.publish_steering(steering)

    def publish_throttle(self, throttle):
        msg = Float32()
        msg.data = throttle
        self.throttle_publisher.publish(msg)

    def publish_brake(self, brake):
        msg = Float32()
        msg.data = brake
        self.brake_publisher.publish(msg)

    def publish_steering(self, steering):
        msg = Float32()
        msg.data = steering
        self.steering_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    can_receiver = CanReceiver()
    can_receiver.receive_can_messages()
    rclpy.spin(can_receiver)
    can_receiver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
