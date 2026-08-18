#!/usr/bin/env python3
import os

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
from ultralytics import YOLO

PERSON_CLASS_ID = 0


def default_model_path():
    return os.path.join(
        get_package_share_directory('four_control'), 'models', 'yolov8n.pt')


class PersonDetectorNode(Node):

    def __init__(self):
        super().__init__('person_detector_node')

        self.declare_parameter('model_path', default_model_path())
        self.declare_parameter('confidence_threshold', 0.4)
        self.declare_parameter('person_height_m', 1.7)
        self.declare_parameter('focal_length_px', 525.0)
        self.declare_parameter('jpeg_quality', 60)
        self.declare_parameter('inference_size', 384)

        model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.person_height_m = self.get_parameter('person_height_m').value
        self.inference_size = self.get_parameter('inference_size').value
        self.focal_length_px = self.get_parameter('focal_length_px').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value

        self.bridge = CvBridge()
        self.model = YOLO(model_path)

        self.image_subscriber = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 1)
        self.camera_info_subscriber = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)

        self.position_publisher = self.create_publisher(
            PointStamped, '/person/position', 10)
        self.alert_publisher = self.create_publisher(
            String, '/person_detected', 10)
        self.annotated_publisher = self.create_publisher(
            Image, '/detection/image_raw', 10)
        self.compressed_publisher = self.create_publisher(
            CompressedImage, '/detection/image_raw/compressed', 10)

    def camera_info_callback(self, msg):
        if msg.k[0] > 0.0:
            self.focal_length_px = msg.k[0]

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        image_height, image_width = cv_image.shape[:2]
        results = self.model(cv_image, imgsz=self.inference_size, verbose=False)

        closest_box = None
        closest_area = 0.0
        for box in results[0].boxes:
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            if cls != PERSON_CLASS_ID or conf < self.confidence_threshold:
                continue
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            area = (x2 - x1) * (y2 - y1)
            cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 200, 255), 1)
            if area > closest_area:
                closest_area = area
                closest_box = (x1, y1, x2, y2, conf)

        if closest_box is not None:
            self.publish_position(closest_box, image_width, cv_image)

        status_text = 'TRACKING' if closest_box is not None else 'SEARCHING...'
        status_color = (0, 0, 255) if closest_box is not None else (0, 255, 0)
        cv2.putText(cv_image, status_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, status_color, 2)

        self.publish_annotated(cv_image, msg.header)

    def publish_position(self, box, image_width, cv_image):
        x1, y1, x2, y2, conf = box
        cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 0, 255), 2)
        label = f'PERSON {conf:.2f}'
        cv2.putText(cv_image, label, (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        box_height_px = max(y2 - y1, 1)
        distance = (self.person_height_m * self.focal_length_px) / box_height_px

        box_center_x = (x1 + x2) / 2.0
        pixel_offset = box_center_x - (image_width / 2.0)
        lateral_offset = (pixel_offset / self.focal_length_px) * distance

        point = PointStamped()
        point.header.stamp = self.get_clock().now().to_msg()
        point.header.frame_id = 'camera_link_optical'
        point.point.x = distance
        point.point.y = lateral_offset
        point.point.z = 0.0
        self.position_publisher.publish(point)

        alert = String()
        alert.data = f'Human detected: {conf:.2f} at {distance:.2f}m'
        self.get_logger().info(alert.data, throttle_duration_sec=5.0)
        self.alert_publisher.publish(alert)

    def publish_annotated(self, cv_image, header):
        image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        image_msg.header = header
        self.annotated_publisher.publish(image_msg)

        ok, encoded = cv2.imencode(
            '.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        if not ok:
            return
        compressed_msg = CompressedImage()
        compressed_msg.header = header
        compressed_msg.format = 'jpeg'
        compressed_msg.data = encoded.tobytes()
        self.compressed_publisher.publish(compressed_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PersonDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
