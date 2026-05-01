#!/usr/bin/env python3
"""
person_detector_node.py
-----------------------
Subscribes to /camera/image_raw, runs YOLOv8n inference,
filters for 'person' class, and publishes:
  - /detections          (vision_msgs/Detection2DArray)
  - /annotated_image     (sensor_msgs/Image)  — bbox overlay for RViz2

ROS2 parameters :
  model_path       path to .pt file  (default: yolov8n.pt — auto-downloads)
  confidence       detection threshold  (default: 0.50)
  device           'cpu' | 'cuda:0'    (default: cpu)
  image_topic      input topic         (default: /camera/image_raw)
  publish_annotated  publish overlay image  (default: true)

Usage:
  ros2 run ros2_person_follower person_detector_node.py

  # override params at runtime
  ros2 run ros2_person_follower person_detector_node.py --ros-args \
      -p confidence:=0.4 -p device:=cuda:0
"""

import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)
from std_msgs.msg import Header

# YOLOv8 — imported lazily so the node starts even if ultralytics isn't
# installed yet (gives a clear error message instead of a crash at import)
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False

# COCO class id for 'person'
PERSON_CLASS_ID = 0

# Colours for the annotated image overlay
BOX_COLOR    = (0, 255, 80)    
TEXT_COLOR   = (255, 255, 255) 
TEXT_BG      = (0, 180, 60)


class PersonDetectorNode(Node):

    def __init__(self):
        super().__init__("person_detector_node")

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter("model_path",        "yolov8n.pt")
        self.declare_parameter("confidence",        0.50)
        self.declare_parameter("device",            "cpu")
        self.declare_parameter("image_topic",       "/camera/image_raw")
        self.declare_parameter("publish_annotated", True)

        self.model_path        = self.get_parameter("model_path").value
        self.conf_threshold    = self.get_parameter("confidence").value
        self.device            = self.get_parameter("device").value
        self.image_topic       = self.get_parameter("image_topic").value
        self.publish_annotated = self.get_parameter("publish_annotated").value

        # ── Validate dependencies ──────────────────────────────────────
        if not YOLO_AVAILABLE:
            self.get_logger().fatal(
                "ultralytics package not found.\n"
                "  Install it:  pip install ultralytics --break-system-packages"
            )
            raise SystemExit(1)

        # ── Load model ────────────────────────────────────────────────
        self.get_logger().info(
            f"Loading YOLOv8 model: {self.model_path}  device={self.device}"
        )
        try:
            self.model = YOLO(self.model_path)
            self.model.to(self.device)
            # Warm-up pass — avoids latency spike on first real frame
            dummy = np.zeros((480, 640, 3), dtype=np.uint8)
            self.model.predict(dummy, verbose=False)
            self.get_logger().info("Model loaded and warmed up ✔")
        except Exception as e:
            self.get_logger().fatal(f"Failed to load YOLO model: {e}")
            raise SystemExit(1)

        # ── cv_bridge ─────────────────────────────────────────────────
        self.bridge = CvBridge()

        # ── Thread lock — YOLO is not thread-safe ─────────────────────
        self._infer_lock = threading.Lock()

        # ── QoS — match Gazebo Harmonic bridge (BEST_EFFORT) ──────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── Subscribers ───────────────────────────────────────────────
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self._image_callback,
            sensor_qos,
        )

        # ── Publishers ────────────────────────────────────────────────
        self.det_pub = self.create_publisher(
            Detection2DArray,
            "/detections",
            reliable_qos,
        )

        if self.publish_annotated:
            self.ann_pub = self.create_publisher(
                Image,
                "/annotated_image",
                reliable_qos,
            )

        # ── Diagnostics ───────────────────────────────────────────────
        self._frame_count   = 0
        self._detect_count  = 0
        self._last_diag_t   = self.get_clock().now()
        self.create_timer(5.0, self._log_diagnostics)

        self.get_logger().info(
            f"PersonDetectorNode ready\n"
            f"  Subscribing : {self.image_topic}\n"
            f"  Publishing  : /detections, /annotated_image\n"
            f"  Confidence  : {self.conf_threshold}\n"
            f"  Device      : {self.device}"
        )

    # ==================================================================
    # IMAGE CALLBACK
    # ==================================================================

    def _image_callback(self, msg: Image):
        # Convert ROS image → OpenCV BGR
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().warn(f"cv_bridge conversion failed: {e}")
            return

        self._frame_count += 1

        # Run inference (thread-safe)
        with self._infer_lock:
            results = self.model.predict(
                cv_image,
                conf=self.conf_threshold,
                classes=[PERSON_CLASS_ID],  # only 'person' — faster
                verbose=False,
            )

        # Parse results into ROS messages
        det_array, person_boxes = self._build_detection_array(
            results, msg.header
        )

        # Publish detections
        self.det_pub.publish(det_array)
        self._detect_count += len(person_boxes)

        # Publish annotated image
        if self.publish_annotated:
            ann_image = self._draw_boxes(cv_image, person_boxes)
            try:
                ann_msg = self.bridge.cv2_to_imgmsg(ann_image, encoding="bgr8")
                ann_msg.header = msg.header
                self.ann_pub.publish(ann_msg)
            except CvBridgeError as e:
                self.get_logger().warn(f"Annotated image publish failed: {e}")

    # ==================================================================
    # BUILD Detection2DArray
    # ==================================================================

    def _build_detection_array(
        self, results, header: Header
    ) -> tuple[Detection2DArray, list]:
        """
        Convert ultralytics Results → vision_msgs/Detection2DArray.
        Returns (det_array, list_of_boxes) where each box is
        (x1, y1, x2, y2, confidence).
        """
        det_array = Detection2DArray()
        det_array.header = header
        person_boxes = []

        for result in results:
            if result.boxes is None:
                continue

            for box in result.boxes:
                cls_id = int(box.cls[0].item())
                if cls_id != PERSON_CLASS_ID:
                    continue

                conf  = float(box.conf[0].item())
                # xyxy — pixel coordinates
                x1, y1, x2, y2 = [float(v) for v in box.xyxy[0].tolist()]

                # centre + size (Detection2D uses this format)
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0
                w  = x2 - x1
                h  = y2 - y1

                det = Detection2D()
                det.header = header

                # Bounding box
                det.bbox.center.position.x = cx
                det.bbox.center.position.y = cy
                det.bbox.size_x = w
                det.bbox.size_y = h

                # Hypothesis
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = str(PERSON_CLASS_ID)
                hyp.hypothesis.score    = conf
                det.results.append(hyp)

                det_array.detections.append(det)
                person_boxes.append((x1, y1, x2, y2, conf))

        return det_array, person_boxes

    # ==================================================================
    # ANNOTATED IMAGE
    # ==================================================================

    def _draw_boxes(
        self, image: np.ndarray, boxes: list
    ) -> np.ndarray:
        """Draw bounding boxes and confidence scores onto a copy of image."""
        out = image.copy()
        img_h, img_w = out.shape[:2]

        for (x1, y1, x2, y2, conf) in boxes:
            ix1, iy1 = int(x1), int(y1)
            ix2, iy2 = int(x2), int(y2)

            # Box
            cv2.rectangle(out, (ix1, iy1), (ix2, iy2), BOX_COLOR, 2)

            # Label background + text
            label   = f"person  {conf:.2f}"
            (tw, th), baseline = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1
            )
            label_y = max(iy1, th + 4)
            cv2.rectangle(
                out,
                (ix1, label_y - th - 4),
                (ix1 + tw + 4, label_y + baseline),
                TEXT_BG, -1,
            )
            cv2.putText(
                out, label,
                (ix1 + 2, label_y - 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                TEXT_COLOR, 1, cv2.LINE_AA,
            )

            # Centroid dot — used by follower_controller_node
            cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
            cv2.circle(out, (cx, cy), 5, (0, 0, 255), -1)

            # Crosshair on image centre — shows angular error at a glance
            mid_x = img_w // 2
            cv2.line(out, (mid_x, 0), (mid_x, img_h), (80, 80, 80), 1)
            cv2.line(out, (cx, cy), (mid_x, cy), (255, 80, 0), 1)

        # Frame + detection counter (top-left HUD)
        hud = f"frame:{self._frame_count}  persons:{len(boxes)}"
        cv2.putText(
            out, hud, (8, 20),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5,
            (200, 200, 200), 1, cv2.LINE_AA,
        )

        return out

    # ==================================================================
    # DIAGNOSTICS
    # ==================================================================

    def _log_diagnostics(self):
        now  = self.get_clock().now()
        dt   = (now - self._last_diag_t).nanoseconds * 1e-9
        hz   = self._frame_count / dt if dt > 0 else 0.0
        self._last_diag_t  = now
        self._frame_count  = 0

        self.get_logger().info(
            f"[detector]  input: {hz:.1f} Hz  |  "
            f"detections last 5s: {self._detect_count}"
        )
        self._detect_count = 0


def main(args=None):
    rclpy.init(args=args)
    try:
        node = PersonDetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()