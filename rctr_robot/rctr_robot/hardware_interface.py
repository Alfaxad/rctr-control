#!/usr/bin/env python3
"""
Hardware Interface Node
Bridges ROS2 with STM32 microcontroller via serial communication
Handles all low-level hardware commands and feedback
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray, String
from geometry_msgs.msg import Twist
import serial
import threading
import time

class HardwareInterface(Node):
    def __init__(self):
        super().__init__('hardware_interface')
        
        # Serial port configuration - adjust port as needed
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        
        # Initialize serial connection
        try:
            self.serial_conn = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f'Connected to STM32 on {port} at {baud} baud')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to STM32: {e}')
            return
        
        # Publishers
        self.encoder_pub = self.create_publisher(
            Int32MultiArray, 'encoder_feedback', 10)
        self.status_pub = self.create_publisher(
            String, 'hardware_status', 10)
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.servo_sub = self.create_subscription(
            Int32MultiArray, 'servo_positions', self.servo_callback, 10)
        self.direct_cmd_sub = self.create_subscription(
            String, 'hardware_command', self.direct_command_callback, 10)
        
        # Start serial reading thread
        self.running = True
        self.serial_thread = threading.Thread(target=self.serial_reader)
        self.serial_thread.start()
        
        # Status timer
        self.status_timer = self.create_timer(0.1, self.request_status)
        
        self.get_logger().info('Hardware interface initialized')
    
    def cmd_vel_callback(self, msg):
        """
        Convert Twist message to differential drive commands
        msg.linear.x: forward velocity (m/s)
        msg.angular.z: angular velocity (rad/s)
        """
        # Robot parameters - adjust based on your robot
        wheel_separation = 0.2  # meters between tracks
        max_linear_speed = 0.5  # m/s
        max_angular_speed = 2.0  # rad/s
        
        # Normalize velocities
        linear_vel = max(-max_linear_speed, min(max_linear_speed, msg.linear.x))
        angular_vel = max(-max_angular_speed, min(max_angular_speed, msg.angular.z))
        
        # Convert to differential drive
        # Left motor speed = linear - (angular * wheel_separation / 2)
        # Right motor speed = linear + (angular * wheel_separation / 2)
        left_speed = linear_vel - (angular_vel * wheel_separation / 2)
        right_speed = linear_vel + (angular_vel * wheel_separation / 2)
        
        # Normalize to -1.0 to 1.0 range
        left_normalized = left_speed / max_linear_speed
        right_normalized = right_speed / max_linear_speed
        
        # Send to STM32
        cmd = f"MOTOR:{left_normalized:.3f},{right_normalized:.3f}\n"
        self.send_command(cmd)
    
    def servo_callback(self, msg):
        """
        Set servo positions for reconfiguration
        msg.data = [servo0_angle, servo1_angle, servo2_angle, servo3_angle]
        Servo mapping:
        - 0: Up/Down front
        - 1: Up/Down rear
        - 2: Left/Right front
        - 3: Left/Right rear
        """
        if len(msg.data) >= 4:
            for i in range(4):
                cmd = f"SERVO:{i},{msg.data[i]}\n"
                self.send_command(cmd)
                time.sleep(0.01)  # Small delay between commands
    
    def direct_command_callback(self, msg):
        """
        Send direct commands to STM32
        """
        self.send_command(msg.data + "\n")
    
    def send_command(self, cmd):
        """
        Send command to STM32 via serial
        """
        try:
            self.serial_conn.write(cmd.encode())
            self.get_logger().debug(f'Sent: {cmd.strip()}')
        except Exception as e:
            self.get_logger().error(f'Failed to send command: {e}')
    
    def request_status(self):
        """
        Request status update from STM32
        """
        self.send_command("STATUS:\n")
    
    def serial_reader(self):
        """
        Background thread to read serial data from STM32
        """
        buffer = ""
        while self.running:
            try:
                if self.serial_conn.in_waiting > 0:
                    data = self.serial_conn.read(self.serial_conn.in_waiting).decode('utf-8', errors='ignore')
                    buffer += data
                    
                    # Process complete lines
                    lines = buffer.split('\n')
                    buffer = lines[-1]  # Keep incomplete line in buffer
                    
                    for line in lines[:-1]:
                        self.process_serial_line(line.strip())
            except Exception as e:
                self.get_logger().error(f'Serial read error: {e}')
            
            time.sleep(0.001)
    
    def process_serial_line(self, line):
        """
        Process incoming serial data from STM32
        """
        if not line:
            return
        
        self.get_logger().debug(f'Received: {line}')
        
        # Parse STATUS response
        if line.startswith("STATUS:"):
            try:
                data = line[7:].split(',')
                if len(data) >= 4:
                    # Publish encoder counts
                    encoder_msg = Int32MultiArray()
                    encoder_msg.data = [int(data[0]), int(data[1])]
                    self.encoder_pub.publish(encoder_msg)
                    
                    # Publish full status
                    self.status_pub.publish(String(data=line))
            except Exception as e:
                self.get_logger().error(f'Failed to parse status: {e}')
        else:
            # Publish other messages as status
            self.status_pub.publish(String(data=line))
    
    def destroy_node(self):
        """
        Clean shutdown
        """
        self.running = False
        if hasattr(self, 'serial_thread'):
            self.serial_thread.join()
        if hasattr(self, 'serial_conn'):
            self.serial_conn.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = HardwareInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
