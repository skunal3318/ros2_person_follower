import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class VisualizationNode(Node):
    def __init__(self):
        super().__init__('visualization_node')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.get_logger().info('Camera obstacle visualization started')


    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w, _ = frame.shape

        # -------------------------
        # 1. Region of Interest (front area)
        # -------------------------
        roi = frame[int(h*0.4):int(h*0.75), :]

        # -------------------------
        # 2. Grayscale + Blur
        # -------------------------
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)

        # -------------------------
        # 3. Edge Detection
        # -------------------------
        edges = cv2.Canny(blur, 50, 150)

        # -------------------------
        # 4. Split regions
        # -------------------------
        left = edges[:, :w//3]
        center = edges[:, w//3:2*w//3]
        right = edges[:, 2*w//3:]

        left_score = np.sum(left)
        center_score = np.sum(center)
        right_score = np.sum(right)

        # -------------------------
        # 5. Decision logic
        # -------------------------
        obstacle_region = "NONE"
        threshold = 5000

        if center_score > threshold:
            obstacle_region = "CENTER"
        elif left_score > threshold:
            obstacle_region = "LEFT"
        elif right_score > threshold:
            obstacle_region = "RIGHT"

        # -------------------------
        # 6. Visualization
        # -------------------------
        roi_color = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)

        cv2.line(roi_color, (w//3, 0), (w//3, roi.shape[0]), (255, 0, 0), 2)
        cv2.line(roi_color, (2*w//3, 0), (2*w//3, roi.shape[0]), (255, 0, 0), 2)

        cv2.putText(
            roi_color,
            f"Obstacle: {obstacle_region}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0),
            2
        )

        # -------------------------
        # 7. Show windows
        # -------------------------
        cv2.imshow("Camera View", frame)
        cv2.imshow("Obstacle ROI (Edges)", roi_color)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = VisualizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
