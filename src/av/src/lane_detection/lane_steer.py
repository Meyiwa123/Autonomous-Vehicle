import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32
from lane_image_processor import process_videos


class SteeringControllerNode(Node):
    def __init__(self):
        super().__init__('steering_controller')

        # Parameters
        self.declare_parameter('distance_threshold', 5)
        # Threshold for distance from center
        self.distance_threshold = self.get_parameter(
            'distance_threshold').value

        # Publishers and Subscribers
        self.zed_image_subscription = self.create_subscription(
            Image, 'zed/zed_node/rgb/image_rect_color', self.image_callback, 10)
        self.image_subscription = self.create_subscription(
            Image, 'image_topic', self.image_callback, 10)
        self.steering_publisher = self.create_publisher(
            Float32, 'lane_steering_angle', 10)
        self.image_publisher = self.create_publisher(
            Image, 'lane_detection_image', 10)

    def image_callback(self, msg):
        try:
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().info('Error converting ROS Image to OpenCV image: %s' % e)
            return

        steering_angle = self.calculate_steering_angle(cv_image)
        self.publish_steering_angle(steering_angle)

    def calculate_steering_angle(self, image):
        cv_final_image, curverad, center_diff = process_videos(image)
        self.image_publisher.publish(
            CvBridge().cv2_to_imgmsg(cv_final_image, "bgr8"))

        steering_angle = 0
        # Check if the distance from center exceeds the threshold
        if abs(center_diff) > self.distance_threshold:
            # Calculate the desired steering angle based on the distance from the center
            steering_angle = math.atan(
                center_diff / curverad)
            # Convert the angle from radians to degrees
            steering_angle = math.degrees(steering_angle)
            # Publish the steering angle
            msg = Float32()
            msg.data = steering_angle
            self.steering_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    steering_controller_node = SteeringControllerNode()
    rclpy.spin(steering_controller_node)
    steering_controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
