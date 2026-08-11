#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.declare_parameter('device_index', 0)
        self.declare_parameter('frame_rate', 30.0)

        device_index = self.get_parameter('device_index').value
        frame_rate = self.get_parameter('frame_rate').value

        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(device_index)
        if not self.cap.isOpened():
            self.get_logger().error(f'Could not open video device {device_index}')

        self.image_publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.timer = self.create_timer(1.0 / frame_rate, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture image', throttle_duration_sec=5.0)
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link_optical'
        self.image_publisher.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
