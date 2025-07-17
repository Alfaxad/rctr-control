#!/usr/bin/env python3
"""
Operator Display Node (Leader Device)
Displays camera feed and telemetry data for operator.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import cv2
import numpy as np
import json
import time
from collections import deque
from pathlib import Path


class OperatorDisplay(Node):
    def __init__(self):
        super().__init__("operator_display")

        # ---------- parameters ----------
        self.declare_parameter("display_telemetry", True)
        self.declare_parameter("record_video", False)
        self.declare_parameter("video_filename", "rctr_recording.mp4")
        self.declare_parameter("loss_timeout", 2.0)   # seconds before “link lost” message
        self.declare_parameter("show_fps", True)

        self.show_telemetry = self.get_parameter("display_telemetry").value
        self.record_video = self.get_parameter("record_video").value
        self.video_filename = self.get_parameter("video_filename").value
        self.loss_timeout = self.get_parameter("loss_timeout").value
        self.show_fps = self.get_parameter("show_fps").value

        # ---------- ROS I/O ----------
        self.image_sub = self.create_subscription(
            CompressedImage,
            "/robot/camera/annotated",
            self.image_callback,
            10,
        )
        self.telemetry_sub = self.create_subscription(
            String,
            "/robot/telemetry",
            self.telemetry_callback,
            10,
        )

        # ---------- GUI & state ----------
        self.window_name = "RCTR Operator View"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 800, 600)

        self.telemetry = {}
        self.last_image_time = time.time()
        self.last_telemetry_time = time.time()

        # FPS helpers
        self.frame_times = deque(maxlen=30)

        # Optional recorder
        self.video_writer = None
        if self.record_video:
            Path(self.video_filename).parent.mkdir(parents=True, exist_ok=True)
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            # writer will be re‑initialised on first frame when frame‑size is known
            self.video_fourcc = fourcc

        self.get_logger().info("Operator display initialised")

    # ------------------------------------------------------------------ callbacks
    def image_callback(self, msg: CompressedImage):
        now = time.time()

        # Decompress JPEG -> BGR image
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warning("Failed to decode image frame")
            return

        h, w = frame.shape[:2]

        # ---------- overlays ----------
        if self.show_telemetry and self.telemetry:
            self.draw_telemetry_overlay(frame)

        if self.show_fps:
            self.update_fps(frame)

        # ---------- link‑loss warnings ----------
        if now - self.last_telemetry_time > self.loss_timeout:
            cv2.putText(
                frame,
                "TELEMETRY LOST",
                (10, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 0, 255),
                3,
            )
        if now - self.last_image_time > self.loss_timeout:
            cv2.putText(
                frame,
                "VIDEO LINK LOST",
                (10, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 0, 255),
                3,
            )

        # Show window
        cv2.imshow(self.window_name, frame)
        key = cv2.waitKey(1)
        if key == 27:  # ESC closes node cleanly
            rclpy.shutdown()

        # ---------- recording ----------
        if self.record_video:
            self.write_video(frame)

        # Update linkage timer
        self.last_image_time = now

    def telemetry_callback(self, msg: String):
        try:
            self.telemetry = json.loads(msg.data)
            self.last_telemetry_time = time.time()
        except json.JSONDecodeError:
            self.get_logger().warning("Bad telemetry JSON")

    # ------------------------------------------------------------------ helpers
    def draw_telemetry_overlay(self, frame):
        """
        Overlay basic telemetry bars & text on frame.
        """
        h, w = frame.shape[:2]
        x0, y0 = 10, h - 140
        line_height = 24
        font = cv2.FONT_HERSHEY_SIMPLEX

        entries = [
            ("L Encoder", str(self.telemetry.get("encoder_left", "?"))),
            ("R Encoder", str(self.telemetry.get("encoder_right", "?"))),
            ("L Motor", f'{self.telemetry.get("motor_left", 0):.2f}'),
            ("R Motor", f'{self.telemetry.get("motor_right", 0):.2f}'),
            ("Roll", f'{self.telemetry.get("roll", 0):.1f}°'),
            ("Pitch", f'{self.telemetry.get("pitch", 0):.1f}°'),
            ("Yaw", f'{self.telemetry.get("yaw", 0):.1f}°'),
        ]

        for i, (label, value) in enumerate(entries):
            y = y0 + i * line_height
            cv2.putText(frame, f"{label}: {value}", (x0, y), font, 0.6, (255, 255, 255), 1)

    def update_fps(self, frame):
        """
        Keep a sliding‑window FPS estimate and display it top‑left.
        """
        now = time.time()
        self.frame_times.append(now)
        if len(self.frame_times) >= 2:
            fps = (len(self.frame_times) - 1) / (self.frame_times[-1] - self.frame_times[0])
            cv2.putText(frame, f"FPS: {fps:4.1f}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)

    def write_video(self, frame):
        if self.video_writer is None:
            h, w = frame.shape[:2]
            self.video_writer = cv2.VideoWriter(
                self.video_filename, self.video_fourcc, 30.0, (w, h)
            )
            if not self.video_writer.isOpened():
                self.get_logger().error("Could not open video file for writing")
                self.record_video = False
                return
        self.video_writer.write(frame)

    # ------------------------------------------------------------------ cleanup
    def destroy_node(self):
        if self.video_writer is not None:
            self.video_writer.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OperatorDisplay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
