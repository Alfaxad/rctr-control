#!/usr/bin/env python3
"""
Robot Hardware Interface Node (Follower Device)
Bridges ROS2 with STM32 microcontroller for teleoperation
Executes commands from operator and sends telemetry back
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Twist
import serial
import threading
import time
import json

class RobotHardwareInterface(Node):
    def __init__(self):
        super().__init__('robot_hardware_interface')
        
        # Serial configuration
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        
        # Initialize serial connection to STM32
        try:
            self.serial_conn = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f'Connected to STM32 on {port}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to STM32: {e}')
            return
        
        # Publishers - telemetry data to send to operator
        self.telemetry_pub = self.create_publisher(
            String, '/robot/telemetry', 10)
        
        # Subscribers - commands from operator
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/robot/cmd_vel', self.cmd_vel_callback, 10)
        self.servo_cmd_sub = self.create_subscription(
            String, '/robot/servo_cmd', self.servo_cmd_callback, 10)
        self.reconfig_sub = self.create_subscription(
            String, '/robot/reconfigure', self.reconfig_callback, 10)
        self.emergency_sub = self.create_subscription(
            String, '/robot/emergency_stop', self.emergency_stop_callback, 10)
        
        # Start serial reading thread
        self.running = True
        self.serial_thread = threading.Thread(target=self.serial_reader)
        self.serial_thread.start()
        
        # Request telemetry timer
        self.telemetry_timer = self.create_timer(0.1, self.request_telemetry)
        
        self.get_logger().info('Robot hardware interface initialized')
    
    def cmd_vel_callback(self, msg):
        """
        Execute velocity commands from operator
        Direct teleoperation control
        """
        # Robot parameters
        wheel_separation = 0.2  # meters
        
        # Convert twist to differential drive
        linear = msg.linear.x
        angular = msg.angular.z
        
        # Calculate wheel speeds
        left_speed = linear - (angular * wheel_separation / 2)
        right_speed = linear + (angular * wheel_separation / 2)
        
        # Normalize to -1 to 1
        max_speed = 0.5  # m/s
        left_norm = max(-1.0, min(1.0, left_speed / max_speed))
        right_norm = max(-1.0, min(1.0, right_speed / max_speed))
        
        # Send to STM32
        cmd = f"MOTOR:{left_norm:.3f},{right_norm:.3f}\n"
        self.send_command(cmd)
    
    def servo_cmd_callback(self, msg):
        """
        Execute servo commands for manual reconfiguration
        """
        try:
            servo_data = json.loads(msg.data)
            channel = servo_data.get('channel', 0)
            angle = servo_data.get('angle', 90)
            
            cmd = f"SERVO:{channel},{angle}\n"
            self.send_command(cmd)
        except Exception as e:
            self.get_logger().error(f'Invalid servo command: {e}')
    
    def reconfig_callback(self, msg):
        """
        Execute predefined reconfigurations
        """
        cmd = f"RECONFIG:{msg.data}\n"
        self.send_command(cmd)
        self.get_logger().info(f'Reconfiguring to: {msg.data}')
    
    def emergency_stop_callback(self, msg):
        """
        Emergency stop - immediately halt motors
        """
        self.send_command("STOP:\n")
        self.get_logger().warn('EMERGENCY STOP ACTIVATED')
    
    def request_telemetry(self):
        """
        Request telemetry update from STM32
        """
        self.send_command("STATUS:\n")
    
    def send_command(self, cmd):
        """
        Send command to STM32
        """
        try:
            self.serial_conn.write(cmd.encode())
            self.get_logger().debug(f'Sent: {cmd.strip()}')
        except Exception as e:
            self.get_logger().error(f'Failed to send command: {e}')
    
    def serial_reader(self):
        """
        Background thread to read telemetry from STM32
        """
        buffer = ""
        while self.running:
            try:
                if self.serial_conn.in_waiting > 0:
                    data = self.serial_conn.read(
                        self.serial_conn.in_waiting).decode('utf-8', errors='ignore')
                    buffer += data
                    
                    lines = buffer.split('\n')
                    buffer = lines[-1]
                    
                    for line in lines[:-1]:
                        self.process_telemetry(line.strip())
            except Exception as e:
                self.get_logger().error(f'Serial read error: {e}')
            
            time.sleep(0.001)
    
    def process_telemetry(self, line):
        """
        Process telemetry data from STM32 and publish to operator
        """
        if not line:
            return
        
        if line.startswith("TELEMETRY:"):
            try:
                # Parse telemetry: encoder1,encoder2,motor1,motor2,roll,pitch,yaw
                data = line[10:].split(',')
                if len(data) >= 7:
                    telemetry = {
                        'encoder_left': int(data[0]),
                        'encoder_right': int(data[1]),
                        'motor_left': float(data[2]),
                        'motor_right': float(data[3]),
                        'roll': float(data[4]),
                        'pitch': float(data[5]),
                        'yaw': float(data[6]),
                        'timestamp': time.time()
                    }
                    
                    # Publish to operator
                    msg = String()
                    msg.data = json.dumps(telemetry)
                    self.telemetry_pub.publish(msg)
            except Exception as e:
                self.get_logger().error(f'Failed to parse telemetry: {e}')
        else:
            # Log other messages
            self.get_logger().info(f'STM32: {line}')
    
    def destroy_node(self):
        """
        Clean shutdown
        """
        self.running = False
        if hasattr(self, 'serial_thread'):
            self.serial_thread.join()
        if hasattr(self, 'serial_conn'):
            # Stop motors before closing
            self.send_command("STOP:\n")
            self.serial_conn.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RobotHardwareInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
