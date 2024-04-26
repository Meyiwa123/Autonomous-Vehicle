#!/usr/bin/env python3
import rclpy
import py_trees
import rclpy.node as Node
from std_msgs.msg import Twist
from av_msgs.msg import DetectedObjectArray


class TrafficLightDetected(py_trees.behaviour.Behaviour):
    DISTANCE_THRESHOLD = 3.0  # Define the distance threshold in meters

    def __init__(self, name: str, node: Node):
        super().__init__(name)
        self._node = node
        self._subscriber = node.create_subscription(
            DetectedObjectArray, 'detected_objects', self.detected_objects_callback, 10)
        self._publisher = node.create_publisher(
            Twist, 'object_detection_brake', 10)
        self._traffic_light_detected = False
        self._distance = 0.0  # Initialize distance to 0

    def update(self) -> py_trees.common.Status:
        if self._traffic_light_detected and self._distance <= self.DISTANCE_THRESHOLD:
            color = self.blackboard.get("traffic_light_color")
            if color == "green":
                # Move forward
                print("Traffic light is green. Moving forward.")
                self._traffic_light_detected = False
                return py_trees.common.Status.SUCCESS
            else:
                # Stop
                print("Traffic light is not green. Stopping.")
                msg = Twist()
                msg.linear.x = 0
                msg.angular.z = 0
                self._publisher.publish(msg)
                self._traffic_light_detected = False
                return py_trees.common.Status.FAILURE
        else:
            # No traffic light detected within threshold distance, continue
            print("No traffic light detected within threshold distance. Continuing.")
            self._traffic_light_detected = False
            return py_trees.common.Status.SUCCESS

    def detected_objects_callback(self, msg: DetectedObjectArray):
        for obj in msg.objects:
            if obj.label == 'traffic light':
                self._traffic_light_detected = True
                self._distance = obj.distance  # Update distance


class StopSignDetected(py_trees.behaviour.Behaviour):
    DISTANCE_THRESHOLD = 2.0  # Define the distance threshold in meters

    def __init__(self, name: str, node: Node):
        super().__init__(name)
        self._node = node
        self._subscriber = node.create_subscription(
            DetectedObjectArray, 'detected_objects', self.detected_objects_callback, 10)
        self._publisher = node.create_publisher(
            Twist, 'object_detection_brake', 10)
        self._stop_sign_detected = False
        self._distance = 0.0  # Initialize distance to 0

    def update(self) -> py_trees.common.Status:
        if self._stop_sign_detected and self._distance <= self.DISTANCE_THRESHOLD:
            # Stop for specified duration
            print("Stop sign detected within threshold distance. Stopping for 3 seconds.")
            msg = Twist()
            msg.linear.x = 0
            msg.angular.z = 0
            self._publisher.publish(msg)
            self._stop_sign_detected = False
            return py_trees.common.Status.SUCCESS
        else:
            # No stop sign detected within threshold distance, continue
            print("No stop sign detected within threshold distance. Continuing.")
            self._stop_sign_detected = False
            return py_trees.common.Status.SUCCESS

    def detected_objects_callback(self, msg: DetectedObjectArray):
        for obj in msg.objects:
            if obj.label == 'stop sign':
                self._stop_sign_detected = True
                self._distance = obj.distance  # Update distance
                return


def create_behavior_tree(node: Node) -> py_trees.behaviour.Behaviour:
    traffic_light_detected = TrafficLightDetected(
        "Traffic Light Detected", node)
    stop_sign_detected = StopSignDetected("Stop Sign Detected", node)

    root = py_trees.composites.Selector("Root")
    root.add_children([traffic_light_detected, stop_sign_detected])
    return root


def main():
    rclpy.init()
    node = rclpy.create_node('behavior_tree_node')
    tree = create_behavior_tree(node)
    tree.setup(timeout=15)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
