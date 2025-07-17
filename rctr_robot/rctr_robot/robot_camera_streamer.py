#!/usr/bin/env python3
"""
Robot Camera Streamer Node (Follower Device)
— USB‑UVC edition for Logitech / generic webcams —

* Grabs frames from /dev/videoN (configurable device_id)
* Resizes (if needed) and JPEG‑encodes
* Publishes:
    - /robot/camera/compressed  (sensor_msgs/CompressedImage)
    - /robot/camera/camera_info (sensor_msgs/CameraInfo)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, CameraInfo
from std_msgs.msg import Header

import cv2
import numpy as np
import time
import threading
from typing import Optional


class RobotCameraStreamer(Node):
    def __init__(self):
        super().__init__("robot_camera_streamer")

        # -------------------- parameters --------------------
        self.declare_parameter("device_id", 0)          # /dev/video0
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 30)
        self.declare_parameter("jpeg_quality", 70)      # 0‑100
        self.declare_parameter("auto_exposure", True)
        self.declare_parameter("exposure_absolute", -1) # -1 = leave unchanged
        self.declare_parameter("gain", -1)              # -1 = leave unchanged

        self.device_id = int(self.get_parameter("device_id").value)
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.fps = int(self.get_parameter("fps").value)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.auto_exposure = bool(self.get_parameter("auto_exposure").value)
        self.exposure_absolute = int(self.get_parameter("exposure_absolute").value)
        self.gain = int(self.get_parameter("gain").value)

        # -------------------- publishers --------------------
        self.image_pub = self.create_publisher(
            CompressedImage, "/robot/camera/compressed", 10
        )
        self.camera_info_pub = self.create_publisher(
            CameraInfo, "/robot/camera/camera_info", 10
        )

        # -------------------- open camera --------------------
        self.cap = cv2.VideoCapture(self.device_id, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().fatal(f"Cannot open /dev/video{self.device_id}")
            raise RuntimeError("Camera open failed")

        # Try to set format
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        # Exposure / gain
        #  For many UVC cams:
        #     CAP_PROP_AUTO_EXPOSURE: 1=en, 0=manual (or reverse depending on backend)
        #     CAP_PROP_EXPOSURE expects log‑scaled negative value on some cameras.
        #     We'll attempt and ignore if not supported.
        try:
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1 if self.auto_exposure else 0)
            if not self.auto_exposure and self.exposure_absolute >= 0:
                self.cap.set(cv2.CAP_PROP_EXPOSURE, float(self.exposure_absolute))
            if self.gain >= 0:
                self.cap.set(cv2.CAP_PROP_GAIN, float(self.gain))
        except Exception as e:
            self.get_logger().warn(f"Exposure/gain settings not supported: {e}")

        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)

        self.get_logger().info(
            f"UVC camera ready @ {actual_w}x{actual_h}  {actual_fps:.1f} fps"
        )

        # Try CUDA JPEG encode if available
        self.cuda_encode = False
        try:
            _ = cv2.cuda.getCudaEnabledDeviceCount()
            if _ > 0:
                self.cuda_encode = True
        except AttributeError:
            pass
        if self.cuda_encode:
            self.get_logger().info("CUDA found – using GPU JPEG encoder")

        # Build CameraInfo stub
        self.camera_info_msg = self._create_camera_info(actual_w, actual_h)

        # Start streaming thread
        self.running = True
        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()

    # --------------------------------------------------------- internals
    def _create_camera_info(self, w, h):
        msg = CameraInfo()
        msg.width = w
        msg.height = h
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0] * 5
        msg.k = [1.0, 0.0, w / 2.0,
                 0.0, 1.0, h / 2.0,
                 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0]
        msg.p = [1.0, 0.0, w / 2.0, 0.0,
                 0.0, 1.0, h / 2.0, 0.0,
                 0.0, 0.0, 1.0, 0.0]
        return msg

    def _capture_loop(self):
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]

        last_time = time.time()
        frame_count = 0

        while self.running and rclpy.ok():
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn("Camera frame dropped")
                continue

            # Encode to JPEG
            if self.cuda_encode:
                gpu = cv2.cuda_GpuMat()
                gpu.upload(frame)
                # cv2.cuda might not have JPEG; fallback if error
                try:
                    _, jpeg = cv2.imencode('.jpg', gpu, encode_param)
                    jpeg_bytes = jpeg.tobytes()
                except Exception:
                    self.cuda_encode = False
                    _, jpeg = cv2.imencode('.jpg', frame, encode_param)
                    jpeg_bytes = jpeg.tobytes()
            else:
                _, jpeg = cv2.imencode('.jpg', frame, encode_param)
                jpeg_bytes = jpeg.tobytes()

            # Build message
            now = self.get_clock().now().to_msg()
            img_msg = CompressedImage()
            img_msg.header = Header()
            img_msg.header.stamp = now
            img_msg.header.frame_id = "camera_link"
            img_msg.format = "jpeg"
            img_msg.data = jpeg_bytes

            # Publish
            self.image_pub.publish(img_msg)
            self.camera_info_msg.header = img_msg.header
            self.camera_info_pub.publish(self.camera_info_msg)

            # FPS logging
            frame_count += 1
            if frame_count >= 60:
                dt = time.time() - last_time
                fps_est = frame_count / dt
                self.get_logger().debug(f"Streaming @ {fps_est:.1f} fps")
                frame_count = 0
                last_time = time.time()

            # Respect target FPS while accounting for blocking read(). Sleep is coarse but OK.
            time.sleep(max(0.0, (1.0 / self.fps) - 0.001))

    # --------------------------------------------------------- teardown
    def destroy_node(self):
        self.running = False
        if hasattr(self, "thread"):
            self.thread.join()
        if hasattr(self, "cap"):
            self.cap.release()
        super().destroy_node()


# ------------------------------------------------------------- main entry
def main(args=None):
    rclpy.init(args=args)
    node = RobotCameraStreamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
