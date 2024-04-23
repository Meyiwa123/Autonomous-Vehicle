import time
import rclpy
import py_trees
import rclpy.node as Node
from av_msgs.msg import DetectedObjectArray


class TrafficLightDetected(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, node: Node):
        super().__init__(name)
        self._node = node
        self._subscriber = node.create_subscription(
            DetectedObjectArray,
            'detected_objects',
            self.detected_objects_callback,
            10
        )
        self._traffic_light_detected = False

    def update(self) -> py_trees.common.Status:
        if self._traffic_light_detected:
            color = self.blackboard.get("traffic_light_color")
            if color == "green":
                # Move forward
                print("Traffic light is green. Moving forward.")
                self._traffic_light_detected = False
                return py_trees.common.Status.SUCCESS
            else:
                # Stop
                print("Traffic light is not green. Stopping.")
                self._traffic_light_detected = False
                return py_trees.common.Status.FAILURE
        else:
            # No traffic light detected, continue
            print("No traffic light detected. Continuing.")
            self._traffic_light_detected = False
            return py_trees.common.Status.SUCCESS

    def detected_objects_callback(self, msg: DetectedObjectArray):
        for obj in msg.objects:
            if obj.label == 'traffic light':
                self._traffic_light_detected = True
                return


class StopSignDetected(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, node: Node):
        super().__init__(name)
        self._node = node
        self._subscriber = node.create_subscription(
            DetectedObjectArray,
            'detected_objects',
            self.detected_objects_callback,
            10
        )
        self._stop_sign_detected = False

    def update(self) -> py_trees.common.Status:
        if self._stop_sign_detected:
            # Stop for specified duration
            print("Stop sign detected. Stopping for 3 seconds.")
            # time.sleep(self.stop_duration)
            self._stop_sign_detected = False
            return py_trees.common.Status.SUCCESS
        else:
            # No stop sign detected, continue
            print("No stop sign detected. Continuing.")
            self._stop_sign_detected = False
            return py_trees.common.Status.SUCCESS

    def detected_objects_callback(self, msg: DetectedObjectArray):
        for obj in msg.objects:
            if obj.label == 'stop sign':
                self._stop_sign_detected = True
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
    rclpy.spin_until_future_complete(node, tree.tick_tock(period_ms=500))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
