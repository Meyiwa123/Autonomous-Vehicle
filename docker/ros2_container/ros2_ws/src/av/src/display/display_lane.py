#!/usr/bin/env python3
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class LaneImageSubscriber(Node):
    def __init__(self):
        super().__init__('lane_image_subscriber')

        # Create a subscriber for the lane image topic
        self.subscription = self.create_subscription(
            Image, 'lane_detection_image', self.image_callback, 10)
        self.subscription  # Prevent unused variable warning

        # Initialize CvBridge
        self.bridge = CvBridge()
        self.cv_image = None

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            self.cv_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(
                f"Error converting ROS Image to OpenCV image: {e}")
            return


def main(args=None):
    rclpy.init(args=args)
    lane_image_subscriber = LaneImageSubscriber()

    # Create OpenCV window
    cv2.namedWindow("Final Lane Image", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Final Lane Image", 800, 600)

    while rclpy.ok():
        rclpy.spin_once(lane_image_subscriber)

        # Check if a new image has been received
        if lane_image_subscriber.cv_image is not None:
            # Display the image using OpenCV
            cv2.imshow("Final Lane Image", lane_image_subscriber.cv_image)

            # Wait for key press to handle window events
            key = cv2.waitKey(1)
            # If 'q' key is pressed, exit loop
            if key & 0xFF == ord('q'):
                break

    # Destroy OpenCV window and cleanup
    cv2.destroyAllWindows()
    lane_image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
