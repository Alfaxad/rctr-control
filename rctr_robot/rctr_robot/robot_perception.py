#!/usr/bin/env python3
"""
Robot Perception Node (Follower Device) – YOLOv8 Edition
-------------------------------------------------------
• Subscribes  : /robot/camera/compressed   (JPEG frames from camera streamer)
• Publishes   : /robot/line_detection      (Float32MultiArray)
                /robot/qr_detection        (String, JSON list of detections)
                /robot/camera/annotated    (CompressedImage, annotated frame)
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, Float32MultiArray
from cv_bridge import CvBridge

import cv2
import numpy as np
import json
import time
from pathlib import Path
from typing import List, Dict

from ultralytics import YOLO
import torch


class RobotPerception(Node):
    def __init__(self):
        super().__init__("robot_perception")

        # -------------------- parameters --------------------
        self.declare_parameter("model_path", "yolov8n.pt")
        self.declare_parameter("device", "cpu")  # "cpu" or "cuda:0"
        self.declare_parameter("conf_threshold", 0.25)
        self.declare_parameter("qr_class_names", ["qr_code", "barcode"])  # names to extract
        self.declare_parameter("process_line_detection", True)
        self.declare_parameter("line_threshold", 50)
        self.declare_parameter("line_roi_bottom", 0.5)  # bottom 50 %

        self.declare_parameter("publish_annotated", True)
        self.declare_parameter("jpeg_quality", 70)

        # grab
        self.model_path = self.get_parameter("model_path").value
        self.device = self.get_parameter("device").value
        self.conf_threshold = float(self.get_parameter("conf_threshold").value)
        self.qr_names = set(self.get_parameter("qr_class_names").get_parameter_value().string_array_value)

        self.do_line = bool(self.get_parameter("process_line_detection").value)
        self.line_threshold = int(self.get_parameter("line_threshold").value)
        self.line_roi = float(self.get_parameter("line_roi_bottom").value)

        self.publish_annotated = bool(self.get_parameter("publish_annotated").value)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)

        # -------------------- YOLO model --------------------
        model_file = Path(self.model_path).expanduser()
        if not model_file.exists():
            self.get_logger().fatal(f"YOLO model not found: {model_file}")
            raise FileNotFoundError(model_file)

        self.get_logger().info(f"Loading YOLOv8 model from {model_file} on {self.device}")
        self.yolo = YOLO(str(model_file))
        try:
            self.yolo.to(self.device)
        except Exception as e:
            self.get_logger().warn(f"Could not move model to {self.device}: {e}")

        # Map class-id -> name for quick lookup
        self.id2name = self.yolo.model.names

        # -------------------- ROS I/O --------------------
        self.bridge = CvBridge()

        self.line_pub = self.create_publisher(Float32MultiArray, "/robot/line_detection", 10)
        self.qr_pub = self.create_publisher(String, "/robot/qr_detection", 10)
        if self.publish_annotated:
            self.annotated_pub = self.create_publisher(
                CompressedImage, "/robot/camera/annotated", 10
            )

        self.image_sub = self.create_subscription(
            CompressedImage, "/robot/camera/compressed", self.image_callback, 10
        )

        self.get_logger().info("Robot perception node initialised")

    # ---------------------------------------------------------------- image path
    def image_callback(self, msg: CompressedImage):
        # JPEG -> BGR
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn("Failed to decode image frame")
            return

        annotated = frame.copy()

        # ----- YOLO inference -----
        t0 = time.time()
        yolo_res = self.yolo.predict(
            frame,
            conf=self.conf_threshold,
            device=self.device,
            imgsz=640,
            verbose=False,
        )[0]  # we sent one frame, so take first element

        qr_detections = self.extract_qr(yolo_res, annotated)
        if qr_detections:
            msg_qr = String()
            msg_qr.data = json.dumps(qr_detections)
            self.qr_pub.publish(msg_qr)

        # ----- optional black-line detection -----
        if self.do_line:
            line_data = self.detect_line(frame)
            self.draw_line_overlay(annotated, line_data)
            self.publish_line(line_data)

        # ----- publish annotated image -----
        if self.publish_annotated:
            self.publish_annotated_image(annotated, msg.header)

        dt_ms = (time.time() - t0) * 1000
        self.get_logger().debug(f"Inference+annot {dt_ms:.1f} ms")

    # ---------------------------------------------------------------- YOLO utils
    def extract_qr(self, yolo_result, annotated_img) -> List[Dict]:
        """
        Pull out detections whose class name matches qr_class_names.
        """
        detections = []
        if yolo_result.boxes is None or len(yolo_result.boxes) == 0:
            return detections

        for box in yolo_result.boxes:
            cls_id = int(box.cls[0])
            name = self.id2name.get(cls_id, str(cls_id))
            if name not in self.qr_names:
                continue

            conf = float(box.conf[0])
            xyxy = box.xyxy[0].cpu().numpy()  # (x1,y1,x2,y2)
            x1, y1, x2, y2 = map(int, xyxy)
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

            detection = {
                "class": name,
                "conf": conf,
                "bbox": [x1, y1, x2, y2],
                "center": [cx, cy],
                "timestamp": time.time(),
            }
            detections.append(detection)

            # ---- draw ----
            cv2.rectangle(annotated_img, (x1, y1), (x2, y2), (0, 255, 255), 2)
            cv2.putText(
                annotated_img,
                f"{name}:{conf:.2f}",
                (x1, y1 - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 255),
                1,
            )
            cv2.circle(annotated_img, (cx, cy), 4, (0, 255, 255), -1)

        return detections

    # ---------------------------------------------------------------- black line
    def detect_line(self, img):
        """
        Basic black-line tracker on lower ROI.
        Returns dictionary compatible with previous API.
        """
        h, w = img.shape[:2]
        roi_top = int(h * (1 - self.line_roi))
        roi = img[roi_top:h, :]

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        _, bin_img = cv2.threshold(blur, self.line_threshold, 255, cv2.THRESH_BINARY_INV)
        bin_img = cv2.morphologyEx(bin_img, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

        contours, _ = cv2.findContours(bin_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        info = {"detected": False, "center_offset": 0.0, "angle": 0.0, "confidence": 0.0}

        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)
            if area > 500:
                vx, vy, x0, y0 = cv2.fitLine(largest, cv2.DIST_L2, 0, 0.01, 0.01)
                angle = float(np.arctan2(vy, vx))
                M = cv2.moments(largest)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    offset = (cx - w / 2) / (w / 2)

                    info.update(
                        {
                            "detected": True,
                            "center_offset": offset,
                            "angle": angle,
                            "confidence": min(1.0, area / (w * h * 0.1)),
                            "cx": cx,
                            "cy": int(M["m01"] / M["m00"]) + roi_top,
                        }
                    )
        return info

    def publish_line(self, line_info):
        arr = Float32MultiArray()
        arr.data = [
            1.0 if line_info["detected"] else 0.0,
            float(line_info["center_offset"]),
            float(line_info["angle"]),
            float(line_info["confidence"]),
        ]
        self.line_pub.publish(arr)

    def draw_line_overlay(self, img, info):
        h, w = img.shape[:2]
        if not info["detected"]:
            cv2.putText(
                img,
                "NO LINE",
                (w // 2 - 60, h - 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2,
            )
            return
        cx = int(w / 2 + info["center_offset"] * w / 2)
        cv2.circle(img, (cx, h - 40), 10, (0, 255, 0), -1)
        cv2.line(img, (w // 2, h - 40), (cx, h - 40), (255, 0, 0), 2)
        cv2.putText(
            img,
            f"off:{info['center_offset']:+.2f}",
            (10, h - 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            1,
        )
        cv2.putText(
            img,
            f"ang:{np.degrees(info['angle']):.1f}",
            (10, h - 70),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            1,
        )

    # ---------------------------------------------------------------- annotated
    def publish_annotated_image(self, img, header):
        # JPEG encode
        _, jpeg = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        msg = CompressedImage()
        msg.header = header
        msg.format = "jpeg"
        msg.data = jpeg.tobytes()
        self.annotated_pub.publish(msg)


# -------------------------------------------------------------------- main
def main(args=None):
    rclpy.init(args=args)
    node = RobotPerception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
