#!/usr/bin/env python3
"""
Operator Display Node (Leader Device)
Displays camera feed and telemetry data for operator
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

class OperatorDisplay(Node):
    def __init__(self):
        super().__init__('operator_display')
        
        # Parameters
        self.declare_parameter('display_telemetry', True)
        self.declare_parameter('record_video', False)
        self.declare_parameter('video_filename', 'rctr_recording.mp4')
        
        # Get parameters
        self.show_telemetry = self.get_parameter('display_telemetry').value
        self.record_video = self.get_parameter('record_video').value
        self.video_filename = self.get_parameter('video_filename').value
        
        # Subscribers
        self.image_sub = self.create_subscription(
            CompressedImage, '/robot/camera/annotated',
            self.image_callback, 10)
        self.telemetry_sub = self.create_subscription(
            String, '/robot/telemetry',
            self.telemetry_callback, 10)
        
        # Display window
        self.window_name = "RCTR Operator View"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 800, 600)
        
        # Telemetry data
        self.telemetry = {
            'encoder_left': 0,
            'encoder_right': 0,
            'motor_left': 0.0,
            'motor_right': 0.0,
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0,
            'timestamp': 0
        }
        
        # FPS calculation
        self.fps_buffer = deque(maxlen=30)
        self.last_frame_time = time.time()
        
        # Video recording
        self.video_writer = None
        if self.record_video:
            self.setup_video_recorder()
        
        # Connection status
        self.last_image_time = time.time()
        self.last_telemetry_time = time.time()
        
        self.get_logger().info('Operator display initialized')
    
    def setup_video_recorder(self):
        """
        Setup video recording
        """
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.video_writer = cv2.VideoWriter(
            self.video_filename, fourcc, 30.0, (800, 600))
        self.get_logger().info(f'Recording video to: {self.video_filename}')
    
    def image_callback(self, msg):
        """
        Display camera feed with overlays
        """
        try:
            # Decompress image
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            # Update connection status
            self.last_image_time = time.time()
            
            # Add telemetry overlay
