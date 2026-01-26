import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time


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

        self.last_time = time.time()
        self.fps = 0.0

        self.get_logger().info('four_control visualization node started')


    def image_callback(self, msg):
        # Convert ROS image to OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w, _ = frame.shape

        # 1. Draw center reference line
        cv2.line(
            frame,
            (w // 2, 0),
            (w // 2, h),
            (0, 255, 0),
            2
        )

        # Draw Region of Interest (ROI)
        roi_y_start = int(h * 0.6)
        roi_y_end = h

        cv2.rectangle(
            frame,
            (0, roi_y_start),
            (w, roi_y_end),
            (255, 0, 0),
            2
        )

       
        # FPS calculation
        current_time = time.time()
        dt = current_time - self.last_time
        if dt > 0:
            self.fps = 1.0 / dt
        self.last_time = current_time

    
        cv2.putText(
            frame,
            f'FPS: {self.fps:.1f}',
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2
        )


        cv2.imshow('Camera View', frame)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = VisualizationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
