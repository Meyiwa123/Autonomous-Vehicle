import cv2
import rclpy
import numpy as np
import supervision as sv
from rclpy.node import Node
from ultralytics import YOLO
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from av.msg import DetectedObject, DetectedObjectArray

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')

        self.model = YOLO("yolov8n.pt")
        self.bridge = CvBridge()

        self.publisher = self.create_publisher(DetectedObjectArray, 'detected_objects', 10)

        self.subscription = self.create_subscription(
            Image, '/zed/zed_node/left/image_rect_color', self.image_callback, 10)
        self.image_subscription = self.create_subscription(
            Image, 'image_topic', self.image_callback, 10)

    def inferrence(self, frame: np.ndarray) -> sv.Detections:
        result = self.model(frame, agnostic_nms=True)[0]
        detections = sv.Detections.from_yolov8(result)
        return detections

    def generate_labels(self, detections: sv.Detections) -> list:
        labels = [
            f"{self.model.model.names[class_id]} {confidence:0.2f}"
            for _, confidence, class_id, _
            in detections
        ]
        return labels

    def annotate_frame(self, frame: np.ndarray, detections: sv.Detections, labels: list) -> np.ndarray:
        box_annotator = sv.BoxAnnotator(
            thickness=2,
            text_thickness=2,
            text_scale=1
        )
        frame = box_annotator.annotate(
            scene=frame,
            detections=detections,
            labels=labels
        )
        return frame

    def publish_detected_objects(self, detections: sv.Detections):
        detected_objects_array = DetectedObjectArray()
        for detection in detections:
            class_id, confidence, _, box = detection
            class_label = self.model.model.names[class_id]
            if class_label in ['traffic light', 'stop sign']:
                detected_object = DetectedObject()
                detected_object.label = class_label
                detected_object.confidence = confidence
                detected_object.x = (box[0] + box[2]) / 2  # X coordinate of the center of the bounding box
                detected_object.y = (box[1] + box[3]) / 2  # Y coordinate of the center of the bounding box
                detected_objects_array.objects.append(detected_object)
        self.publisher.publish(detected_objects_array)

    def image_callback(self, msg: Image) -> None:
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        detections = self.inferrence(cv_image)
        labels = self.generate_labels(detections)
        annotated_image = self.annotate_frame(cv_image, detections, labels)

        self.publish_detected_objects(detections)

        cv2.imshow("Object Detection", annotated_image)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = ObjectDetectionNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
