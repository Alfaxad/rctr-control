#!/usr/bin/env python3
"""
Perception Node using YOLOv8 nano
Detects track lines, obstacles, and challenge markers
Publishes detection results for control decisions
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String, Float32MultiArray, Int32
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import json
import time

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        
        # Parameters
        self.declare_parameter('use_compressed', True)
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('yolo_model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('visualize', True)
        
        # Get parameters
        use_compressed = self.get_parameter('use_compressed').value
        camera_topic = self.get_parameter('camera_topic').value
        model_path = self.get_parameter('yolo_model_path').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.visualize = self.get_parameter('visualize').value
        
        # Initialize YOLO model
        try:
            self.model = YOLO(model_path)
            self.get_logger().info(f'Loaded YOLO model: {model_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {e}')
            return
        
        # Define challenge classes (customize based on your training)
        self.challenge_classes = {
            'line': 0,
            'door': 1,
            'slope': 2,
            'ditch': 3,
            'qr_code': 4,
            'bump': 5,
            'stairs': 6,
            'obstacle': 7,
            'uneven_terrain': 8,
            'roller': 9,
            'traffic_light': 10,
            'barrier': 11
        }
        
        # OpenCV bridge
        self.bridge = CvBridge()
        
        # Publishers
        self.line_pub = self.create_publisher(
            Float32MultiArray, 'line_detection', 10)
        self.challenge_pub = self.create_publisher(
            String, 'challenge_detection', 10)
        self.viz_pub = self.create_publisher(
            Image, 'perception_viz', 10)
        
        # Subscriber
        if use_compressed:
            self.image_sub = self.create_subscription(
                CompressedImage, camera_topic + '/compressed',
                self.compressed_image_callback, 10)
        else:
            self.image_sub = self.create_subscription(
                Image, camera_topic, self.image_callback, 10)
        
        # Line detection parameters
        self.line_roi_top = 0.5  # Start looking at bottom half of image
        self.line_threshold = 50  # Binary threshold for black line
        
        # FPS tracking
        self.last_time = time.time()
        self.fps = 0
        
        self.get_logger().info('Perception node initialized')
    
    def image_callback(self, msg):
        """Process raw image"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.process_image(cv_image)
        except Exception as e:
            self.get_logger().error(f'Image conversion error: {e}')
    
    def compressed_image_callback(self, msg):
        """Process compressed image"""
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.process_image(cv_image)
        except Exception as e:
            self.get_logger().error(f'Compressed image error: {e}')
    
    def process_image(self, image):
        """
        Main image processing pipeline
        1. Line detection using traditional CV
        2. Challenge detection using YOLO
        """
        # FPS calculation
        current_time = time.time()
        self.fps = 1.0 / (current_time - self.last_time)
        self.last_time = current_time
        
        # Get image dimensions
        height, width = image.shape[:2]
        
        # 1. Line Detection (traditional CV for reliability)
        line_data = self.detect_line(image)
        
        # 2. YOLO Detection for challenges
        detections = self.detect_challenges(image)
        
        # 3. Visualization
        if self.visualize:
            viz_image = self.visualize_detections(
                image, line_data, detections)
            
            # Add FPS
            cv2.putText(viz_image, f'FPS: {self.fps:.1f}',
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                       1, (0, 255, 0), 2)
            
            # Publish visualization
            viz_msg = self.bridge.cv2_to_imgmsg(viz_image, "bgr8")
            self.viz_pub.publish(viz_msg)
    
    def detect_line(self, image):
        """
        Detect black line for path following
        Returns line center offset and angle
        """
        height, width = image.shape[:2]
        
        # Define ROI (bottom half of image)
        roi_top = int(height * self.line_roi_top)
        roi = image[roi_top:height, :]
        
        # Convert to grayscale
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Binary threshold for black line
        _, binary = cv2.threshold(blurred, self.line_threshold, 255,
                                 cv2.THRESH_BINARY_INV)
        
        # Find contours
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL,
                                      cv2.CHAIN_APPROX_SIMPLE)
        
        line_data = {
            'detected': False,
            'center_offset': 0.0,  # Normalized -1 to 1
            'angle': 0.0,          # Line angle in radians
            'confidence': 0.0
        }
        
        if contours:
            # Find largest contour (main line)
            largest_contour = max(contours, key=cv2.contourArea)
            
            # Minimum area check
            if cv2.contourArea(largest_contour) > 500:
                # Fit line to contour
                [vx, vy, x, y] = cv2.fitLine(largest_contour,
                                            cv2.DIST_L2, 0, 0.01, 0.01)
                
                # Calculate line angle
                line_data['angle'] = np.arctan2(vy, vx)[0]
                
                # Get line center
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"]) + roi_top
                    
                    # Normalize center offset (-1 to 1)
                    line_data['center_offset'] = (cx - width/2) / (width/2)
                    line_data['detected'] = True
                    line_data['confidence'] = min(1.0,
                        cv2.contourArea(largest_contour) / (width * height * 0.1))
        
        # Publish line detection
        line_msg = Float32MultiArray()
        line_msg.data = [
            float(line_data['detected']),
            line_data['center_offset'],
            line_data['angle'],
            line_data['confidence']
        ]
        self.line_pub.publish(line_msg)
        
        return line_data
    
    def detect_challenges(self, image):
        """
        Detect challenges using YOLO
        """
        # Run YOLO inference
        results = self.model(image, conf=self.conf_threshold, verbose=False)
        
        detections = []
        if len(results) > 0:
            result = results[0]
            if result.boxes is not None:
                for box in result.boxes:
                    # Get detection info
                    cls_id = int(box.cls[0])
                    confidence = float(box.conf[0])
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    
                    # Get class name
                    class_name = self.model.names.get(cls_id, 'unknown')
                    
                    detection = {
                        'class': class_name,
                        'class_id': cls_id,
                        'confidence': confidence,
                        'bbox': [x1, y1, x2, y2],
                        'center': [(x1+x2)/2, (y1+y2)/2],
                        'area': (x2-x1) * (y2-y1)
                    }
                    detections.append(detection)
        
        # Publish detections
        if detections:
            detection_msg = String()
            detection_msg.data = json.dumps(detections)
            self.challenge_pub.publish(detection_msg)
        
        return detections
    
    def visualize_detections(self, image, line_data, detections):
        """
        Visualize all detections on image
        """
        viz_image = image.copy()
        height, width = viz_image.shape[:2]
        
        # Draw line detection
        if line_data['detected']:
            # Draw line center
            center_x = int(width/2 + line_data['center_offset'] * width/2)
            cv2.line(viz_image, (center_x, height-50),
                    (center_x, height), (0, 255, 0), 3)
            
            # Draw offset indicator
            cv2.arrowedLine(viz_image, (width//2, height-30),
                           (center_x, height-30), (255, 0, 0), 2)
            
            # Show offset value
            cv2.putText(viz_image, f"Offset: {line_data['center_offset']:.2f}",
                       (10, height-10), cv2.FONT_HERSHEY_SIMPLEX,
                       0.5, (255, 255, 255), 1)
        
        # Draw YOLO detections
        for det in detections:
            x1, y1, x2, y2 = [int(x) for x in det['bbox']]
            
            # Color based on class
            color = self.get_class_color(det['class'])
            
            # Draw bounding box
            cv2.rectangle(viz_image, (x1, y1), (x2, y2), color, 2)
            
            # Draw label
            label = f"{det['class']}: {det['confidence']:.2f}"
            label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX,
                                           0.5, 1)
            cv2.rectangle(viz_image, (x1, y1-20),
                         (x1 + label_size[0], y1), color, -1)
            cv2.putText(viz_image, label, (x1, y1-5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return viz_image
    
    def get_class_color(self, class_name):
        """Get consistent color for each class"""
        colors = {
            'line': (0, 0, 0),
            'door': (255, 0, 0),
            'slope': (0, 255, 0),
            'ditch': (0, 0, 255),
            'qr_code': (255, 255, 0),
            'bump': (255, 0, 255),
            'stairs': (0, 255, 255),
            'obstacle': (128, 0, 128),
            'uneven_terrain': (128, 128, 0),
            'roller': (0, 128, 128),
            'traffic_light': (255, 128, 0),
            'barrier': (128, 255, 0)
        }
        return colors.get(class_name, (128, 128, 128))

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
