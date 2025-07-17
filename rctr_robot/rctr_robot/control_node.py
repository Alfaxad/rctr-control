#!/usr/bin/env python3
"""
Control Node - Main robot control logic
Implements multi-PID control for line following and navigation
Handles high-level decision making based on perception
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, String, Int32MultiArray
import json
import numpy as np
import time

class PIDController:
    """Generic PID controller implementation"""
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, min_output=-1.0, max_output=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_output = min_output
        self.max_output = max_output
        
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = time.time()
    
    def update(self, error, current_time=None):
        """Calculate PID output"""
        if current_time is None:
            current_time = time.time()
        
        dt = current_time - self.prev_time
        if dt <= 0:
            dt = 0.001
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term
        self.integral += error * dt
        i_term = self.ki * self.integral
        
        # Derivative term
        d_term = self.kd * (error - self.prev_error) / dt
        
        # Total output
        output = p_term + i_term + d_term
        
        # Clamp output
        output = max(self.min_output, min(self.max_output, output))
        
        # Update state
        self.prev_error = error
        self.prev_time = current_time
        
        return output
    
    def reset(self):
        """Reset controller state"""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = time.time()

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        
        # Parameters
        self.declare_parameter('control_rate', 50.0)  # Hz
        self.declare_parameter('base_speed', 0.3)    # m/s
        self.declare_parameter('max_speed', 0.5)     # m/s
        self.declare_parameter('max_angular', 2.0)   # rad/s
        
        # PID gains for line following
        self.declare_parameter('line_kp', 2.0)
        self.declare_parameter('line_ki', 0.1)
        self.declare_parameter('line_kd', 0.5)
        
        # PID gains for angle correction
        self.declare_parameter('angle_kp', 1.0)
        self.declare_parameter('angle_ki', 0.05)
        self.declare_parameter('angle_kd', 0.2)
        
        # Get parameters
        self.control_rate = self.get_parameter('control_rate').value
        self.base_speed = self.get_parameter('base_speed').value
        self.max_speed = self.get_parameter('max_speed').value
        self.max_angular = self.get_parameter('max_angular').value
        
        # Initialize PID controllers
        self.line_pid = PIDController(
            self.get_parameter('line_kp').value,
            self.get_parameter('line_ki').value,
            self.get_parameter('line_kd').value,
            -self.max_angular, self.max_angular
        )
        
        self.angle_pid = PIDController(
            self.get_parameter('angle_kp').value,
            self.get_parameter('angle_ki').value,
            self.get_parameter('angle_kd').value,
            -self.max_angular, self.max_angular
        )
        
        # State variables
        self.current_state = 'LINE_FOLLOWING'
        self.line_detected = False
        self.line_offset = 0.0
        self.line_angle = 0.0
        self.line_confidence = 0.0
        self.current_challenge = None
        self.challenge_detections = []
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.servo_pub = self.create_publisher(
            Int32MultiArray, 'servo_positions', 10)
        self.state_pub = self.create_publisher(String, 'robot_state', 10)
        
        # Subscribers
        self.line_sub = self.create_subscription(
            Float32MultiArray, 'line_detection',
            self.line_callback, 10)
        self.challenge_sub = self.create_subscription(
            String, 'challenge_detection',
            self.challenge_callback, 10)
        
        # Control timer
        self.control_timer = self.create_timer(
            1.0 / self.control_rate, self.control_loop)
        
        # Challenge handling parameters
        self.challenge_handlers = {
            'door': self.handle_door,
            'slope': self.handle_slope,
            'ditch': self.handle_ditch,
            'qr_code': self.handle_qr_code,
            'bump': self.handle_bump,
            'stairs': self.handle_stairs,
            'obstacle': self.handle_obstacle,
            'uneven_terrain': self.handle_uneven_terrain,
            'roller': self.handle_roller,
            'traffic_light': self.handle_traffic_light,
            'barrier': self.handle_barrier
        }
        
        # Reconfiguration presets (servo angles)
        self.reconfig_presets = {
            'neutral': [90, 90, 90, 90],
            'climb_up': [45, 135, 90, 90],    # Bend up
            'climb_down': [135, 45, 90, 90],  # Bend down
            'turn_left': [90, 90, 45, 135],   # Bend left
            'turn_right': [90, 90, 135, 45],  # Bend right
            'bridge': [60, 120, 90, 90],      # Arch shape for ditch
            'compress': [70, 110, 70, 110]    # Compress for tight spaces
        }
        
        self.get_logger().info('Control node initialized')
    
    def line_callback(self, msg):
        """Update line detection data"""
        if len(msg.data) >= 4:
            self.line_detected = bool(msg.data[0])
            self.line_offset = msg.data[1]
            self.line_angle = msg.data[2]
            self.line_confidence = msg.data[3]
    
    def challenge_callback(self, msg):
        """Update challenge detections"""
        try:
            self.challenge_detections = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f'Failed to parse challenges: {e}')
    
    def control_loop(self):
        """Main control loop"""
        # Publish current state
        state_msg = String()
        state_msg.data = json.dumps({
            'state': self.current_state,
            'line_detected': self.line_detected,
            'line_offset': self.line_offset,
            'current_challenge': self.current_challenge
        })
        self.state_pub.publish(state_msg)
        
        # State machine
        if self.current_state == 'LINE_FOLLOWING':
            self.line_following_control()
            
            # Check for challenges
            if self.challenge_detections:
                # Priority: closest/largest detection
                challenge = max(self.challenge_detections,
                              key=lambda x: x['area'])
                
                # If challenge is close enough, handle it
                if challenge['area'] > 10000:  # Threshold for proximity
                    self.current_challenge = challenge['class']
                    self.current_state = 'CHALLENGE_HANDLING'
        
        elif self.current_state == 'CHALLENGE_HANDLING':
            if self.current_challenge in self.challenge_handlers:
                # Call specific challenge handler
                completed = self.challenge_handlers[self.current_challenge]()
                
                if completed:
                    self.current_challenge = None
                    self.current_state = 'LINE_FOLLOWING'
                    self.reconfigure('neutral')
            else:
                # Unknown challenge, try to continue
                self.current_state = 'LINE_FOLLOWING'
        
        elif self.current_state == 'STOPPED':
            # Emergency stop
            self.publish_velocity(0, 0)
    
    def line_following_control(self):
        """PID-based line following"""
        if not self.line_detected:
            # No line detected - search pattern
            self.search_for_line()
            return
        
        # Calculate control signals
        # Line offset control (keep line centered)
        angular_vel = self.line_pid.update(self.line_offset)
        
        # Add angle correction (keep parallel to line)
        angular_vel += self.angle_pid.update(self.line_angle) * 0.3
        
        # Speed reduction based on turning
        speed_factor = 1.0 - abs(angular_vel) / self.max_angular * 0.5
        linear_vel = self.base_speed * speed_factor
        
        # Publish velocity command
        self.publish_velocity(linear_vel, angular_vel)
    
    def search_for_line(self):
        """Search pattern when line is lost"""
        # Slow rotation to find line
        self.publish_velocity(0.0, 0.5)
    
    def publish_velocity(self, linear, angular):
        """Publish velocity command"""
        twist = Twist()
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)
        self.cmd_vel_pub.publish(twist)
    
    def reconfigure(self, preset_name):
        """Set robot reconfiguration"""
        if preset_name in self.reconfig_presets:
            servo_msg = Int32MultiArray()
            servo_msg.data = self.reconfig_presets[preset_name]
            self.servo_pub.publish(servo_msg)
            time.sleep(0.5)  # Wait for reconfiguration
    
    # Challenge Handlers
    def handle_door(self):
        """Handle door challenge"""
        self.get_logger().info('Handling door')
        # Simple approach: push through
        self.publish_velocity(self.max_speed, 0)
        time.sleep(2.0)
        return True
    
    def handle_slope(self):
        """Handle slope challenge"""
        self.get_logger().info('Handling slope')
        # Reconfigure for climbing
        self.reconfigure('climb_up')
        # Increase speed for momentum
        self.publish_velocity(self.max_speed, 0)
        time.sleep(3.0)
        self.reconfigure('neutral')
        return True
    
    def handle_ditch(self):
        """Handle ditch challenge"""
        self.get_logger().info('Handling ditch')
        # Bridge configuration
        self.reconfigure('bridge')
        # Cross carefully
        self.publish_velocity(self.base_speed * 0.5, 0)
        time.sleep(4.0)
        self.reconfigure('neutral')
        return True
    
    def handle_qr_code(self):
        """Handle QR code challenge"""
        self.get_logger().info('Handling QR code')
        # Stop and process (actual QR reading would go here)
        self.publish_velocity(0, 0)
        time.sleep(2.0)
        # Continue
        return True
    
    def handle_bump(self):
        """Handle bump challenge"""
        self.get_logger().info('Handling bump')
        # Similar to slope but shorter
        self.reconfigure('climb_up')
        self.publish_velocity(self.base_speed, 0)
        time.sleep(2.0)
        self.reconfigure('neutral')
        return True
    
    def handle_stairs(self):
        """Handle stairs challenge"""
        self.get_logger().info('Handling stairs')
        # Multi-step climbing
        for i in range(3):  # Assuming 3 steps
            self.reconfigure('climb_up')
            self.publish_velocity(self.base_speed * 0.7, 0)
            time.sleep(1.5)
            self.reconfigure('neutral')
            time.sleep(0.5)
        return True
    
    def handle_obstacle(self):
        """Handle obstacle/object transport"""
        self.get_logger().info('Handling obstacle')
        # This would require gripper control
        # For now, just navigate around
        self.publish_velocity(0, self.max_angular * 0.5)
        time.sleep(2.0)
        self.publish_velocity(self.base_speed, 0)
        time.sleep(2.0)
        return True
    
    def handle_uneven_terrain(self):
        """Handle uneven terrain"""
        self.get_logger().info('Handling uneven terrain')
        # Slow and steady
        self.publish_velocity(self.base_speed * 0.3, 0)
        time.sleep(5.0)
        return True
    
    def handle_roller(self):
        """Handle roller/pipe"""
        self.get_logger().info('Handling roller')
        # Compress to increase ground pressure
        self.reconfigure('compress')
        self.publish_velocity(self.base_speed * 0.5, 0)
        time.sleep(3.0)
        self.reconfigure('neutral')
        return True
    
    def handle_traffic_light(self):
        """Handle traffic light"""
        self.get_logger().info('Handling traffic light')
        # Would need actual light detection
        # For now, wait then continue
        self.publish_velocity(0, 0)
        time.sleep(3.0)
        return True
    
    def handle_barrier(self):
        """Handle barrier"""
        self.get_logger().info('Handling barrier')
        # Would need RF communication
        # For now, just wait
        self.publish_velocity(0, 0)
        time.sleep(2.0)
        return True

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
