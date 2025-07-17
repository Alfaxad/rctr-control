#!/usr/bin/env python3
"""
Operator Control Node (Leader Device)
Captures keyboard/joystick input and sends commands to robot
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import pygame
import json
import threading
import time

class OperatorControl(Node):
    def __init__(self):
        super().__init__('operator_control')
        
        # Parameters
        self.declare_parameter('use_joystick', False)
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 2.0)
        self.declare_parameter('speed_increment', 0.05)
        
        # Get parameters
        self.use_joystick = self.get_parameter('use_joystick').value
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.speed_inc = self.get_parameter('speed_increment').value
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/robot/cmd_vel', 10)
        self.servo_pub = self.create_publisher(
            String, '/robot/servo_cmd', 10)
        self.reconfig_pub = self.create_publisher(
            String, '/robot/reconfigure', 10)
        self.emergency_pub = self.create_publisher(
            String, '/robot/emergency_stop', 10)
        
        # Control state
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.servo_angles = [90, 90, 90, 90]
        self.control_mode = 'DRIVE'  # DRIVE or SERVO
        
        # Initialize pygame for input
        pygame.init()
        
        if self.use_joystick:
            # Initialize joystick
            pygame.joystick.init()
            if pygame.joystick.get_count() > 0:
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                self.get_logger().info(f'Joystick initialized: {self.joystick.get_name()}')
            else:
                self.get_logger().warn('No joystick found, using keyboard')
                self.use_joystick = False
        
        # Start input thread
        self.running = True
        self.input_thread = threading.Thread(target=self.input_loop)
        self.input_thread.start()
        
        # Control publish timer
        self.control_timer = self.create_timer(0.05, self.publish_control)
        
        self.print_help()
        self.get_logger().info('Operator control initialized')
    
    def print_help(self):
        """
        Print control instructions
        """
        help_text = """
        === RCTR TELEOPERATION CONTROLS ===
        
        DRIVING MODE (Tab to switch modes):
        W/↑ : Forward          S/↓ : Backward
        A/← : Turn Left        D/→ : Turn Right
        Q   : Strafe Left      E   : Strafe Right
        Space : Stop           Shift : Boost
        
        RECONFIGURATION PRESETS:
        1 : Neutral            2 : Climb Up
        3 : Climb Down         4 : Turn Left
        5 : Turn Right         6 : Bridge Mode
        
        SERVO MODE (Tab to switch):
        1-4 : Select Servo     ↑/↓ : Adjust Angle
        
        OTHER:
        ESC : Emergency Stop   H : Show Help
        Tab : Toggle Mode      R : Reset
        """
        print(help_text)
    
    def input_loop(self):
        """
        Main input processing loop
        """
        # Create a small pygame window to capture events
        screen = pygame.display.set_mode((400, 300))
        pygame.display.set_caption("RCTR Teleoperation Control")
        clock = pygame.time.Clock()
        
        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                elif event.type == pygame.KEYDOWN:
                    self.handle_keydown(event)
                elif event.type == pygame.KEYUP:
                    self.handle_keyup(event)
            
            # Process continuous key states
            keys = pygame.key.get_pressed()
            self.process_keys(keys)
            
            # Process joystick if available
            if self.use_joystick and hasattr(self, 'joystick'):
                self.process_joystick()
            
            # Update display
            self.update_display(screen)
            
            clock.tick(30)  # 30 Hz input loop
        
        pygame.quit()
    
    def handle_keydown(self, event):
        """
        Handle key press events
        """
        if event.key == pygame.K_ESCAPE:
            # Emergency stop
            self.emergency_stop()
        
        elif event.key == pygame.K_TAB:
            # Toggle control mode
            self.control_mode = 'SERVO' if self.control_mode == 'DRIVE' else 'DRIVE'
            self.get_logger().info(f'Control mode: {self.control_mode}')
        
        elif event.key == pygame.K_h:
            # Show help
            self.print_help()
        
        elif event.key == pygame.K_r:
            # Reset
            self.linear_speed = 0.0
            self.angular_speed = 0.0
            self.servo_angles = [90, 90, 90, 90]
        
        # Reconfiguration presets (in drive mode)
        elif self.control_mode == 'DRIVE':
            if event.key == pygame.K_1:
                self.send_reconfiguration('NEUTRAL')
            elif event.key == pygame.K_2:
                self.send_reconfiguration('CLIMB_UP')
            elif event.key == pygame.K_3:
                self.send_reconfiguration('CLIMB_DOWN')
            elif event.key == pygame.K_4:
                self.send_reconfiguration('TURN_LEFT')
            elif event.key == pygame.K_5:
                self.send_reconfiguration('TURN_RIGHT')
            elif event.key == pygame.K_6:
                self.send_reconfiguration('BRIDGE')
    
    def handle_keyup(self, event):
        """
        Handle key release events
        """
        # Stop on release of movement keys
        if self.control_mode == 'DRIVE':
            if event.key in [pygame.K_w, pygame.K_s, pygame.K_UP, pygame.K_DOWN]:
                self.linear_speed = 0.0
            elif event.key in [pygame.K_a, pygame.K_d, pygame.K_LEFT, pygame.K_RIGHT]:
                self.angular_speed = 0.0
    
    def process_keys(self, keys):
        """
        Process continuous keyboard input
        """
        if self.control_mode == 'DRIVE':
            # Linear motion
            if keys[pygame.K_w] or keys[pygame.K_UP]:
                self.linear_speed = min(self.max_linear, 
                                      self.linear_speed + self.speed_inc)
            elif keys[pygame.K_s] or keys[pygame.K_DOWN]:
                self.linear_speed = max(-self.max_linear, 
                                      self.linear_speed - self.speed_inc)
            else:
                # Decay to zero
                if abs(self.linear_speed) > 0.01:
                    self.linear_speed *= 0.9
                else:
                    self.linear_speed = 0.0
            
            # Angular motion
            if keys[pygame.K_a] or keys[pygame.K_LEFT]:
                self.angular_speed = min(self.max_angular, 
                                       self.angular_speed + self.speed_inc * 2)
            elif keys[pygame.K_d] or keys[pygame.K_RIGHT]:
                self.angular_speed = max(-self.max_angular, 
                                       self.angular_speed - self.speed_inc * 2)
            else:
                # Decay to zero
                if abs(self.angular_speed) > 0.01:
                    self.angular_speed *= 0.9
                else:
                    self.angular_speed = 0.0
            
            # Boost mode
            if keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]:
                self.linear_speed *= 1.5
                self.linear_speed = max(-self.max_linear, 
                                      min(self.max_linear, self.linear_speed))
            
            # Emergency stop
            if keys[pygame.K_SPACE]:
                self.linear_speed = 0.0
                self.angular_speed = 0.0
        
        elif self.control_mode == 'SERVO':
            # Servo control mode
            # Select servo with number keys
            for i in range(4):
                if keys[pygame.K_1 + i]:
                    self.selected_servo = i
            
            # Adjust servo angle
            if hasattr(self, 'selected_servo'):
                if keys[pygame.K_UP]:
                    self.servo_angles[self.selected_servo] = min(180,
                        self.servo_angles[self.selected_servo] + 1)
                    self.send_servo_command(self.selected_servo, 
                                          self.servo_angles[self.selected_servo])
                elif keys[pygame.K_DOWN]:
                    self.servo_angles[self.selected_servo] = max(0,
                        self.servo_angles[self.selected_servo] - 1)
                    self.send_servo_command(self.selected_servo, 
                                          self.servo_angles[self.selected_servo])
    
    def process_joystick(self):
        """
        Process joystick input
        """
        if self.control_mode == 'DRIVE':
            # Get axis values
            linear_axis = -self.joystick.get_axis(1)  # Y-axis (inverted)
            angular_axis = -self.joystick.get_axis(0)  # X-axis (inverted)
            
            # Apply deadzone
            deadzone = 0.1
            if abs(linear_axis) < deadzone:
                linear_axis = 0
            if abs(angular_axis) < deadzone:
                angular_axis = 0
            
            # Set speeds
            self.linear_speed = linear_axis * self.max_linear
            self.angular_speed = angular_axis * self.max_angular
            
            # Buttons for reconfiguration
            if self.joystick.get_button(0):  # A button
                self.send_reconfiguration('NEUTRAL')
            elif self.joystick.get_button(1):  # B button
                self.emergency_stop()
    
    def publish_control(self):
        """
        Publish control commands
        """
        if self.control_mode == 'DRIVE':
            # Publish velocity command
            twist = Twist()
            twist.linear.x = self.linear_speed
            twist.angular.z = self.angular_speed
            self.cmd_vel_pub.publish(twist)
    
    def send_servo_command(self, channel, angle):
        """
        Send servo position command
        """
        msg = String()
        msg.data = json.dumps({
            'channel': channel,
            'angle': angle
        })
        self.servo_pub.publish(msg)
    
    def send_reconfiguration(self, config_name):
        """
        Send reconfiguration preset command
        """
        msg = String()
        msg.data = config_name
        self.reconfig_pub.publish(msg)
        self.get_logger().info(f'Reconfiguration: {config_name}')
    
    def emergency_stop(self):
        """
        Send emergency stop command
        """
        msg = String()
        msg.data = "STOP"
        self.emergency_pub.publish(msg)
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.get_logger().warn('EMERGENCY STOP!')
    
    def update_display(self, screen):
        """
        Update pygame display with status
        """
        screen.fill((0, 0, 0))
        
        font = pygame.font.Font(None, 36)
        small_font = pygame.font.Font(None, 24)
        
        # Title
        title = font.render("RCTR Teleoperation", True, (255, 255, 255))
        screen.blit(title, (50, 20))
        
        # Mode
        mode_text = f"Mode: {self.control_mode}"
        mode_surface = small_font.render(mode_text, True, (0, 255, 0))
        screen.blit(mode_surface, (50, 70))
        
        # Speed display
        linear_text = f"Linear: {self.linear_speed:.2f} m/s"
        angular_text = f"Angular: {self.angular_speed:.2f} rad/s"
        
        linear_surface = small_font.render(linear_text, True, (255, 255, 255))
        angular_surface = small_font.render(angular_text, True, (255, 255, 255))
        
        screen.blit(linear_surface, (50, 110))
        screen.blit(angular_surface, (50, 140))
        
        # Servo angles
        if self.control_mode == 'SERVO':
            for i in range(4):
                servo_text = f"Servo {i}: {self.servo_angles[i]}°"
                color = (255, 255, 0) if hasattr(self, 'selected_servo') and self.selected_servo == i else (255, 255, 255)
                servo_surface = small_font.render(servo_text, True, color)
                screen.blit(servo_surface, (50, 180 + i * 30))
        
        # Instructions
        inst_text = "Press H for help, ESC for emergency stop"
        inst_surface = small_font.render(inst_text, True, (128, 128, 128))
        screen.blit(inst_surface, (50, 260))
        
        pygame.display.flip()
    
    def destroy_node(self):
        """
        Clean shutdown
        """
        self.running = False
        if hasattr(self, 'input_thread'):
            self.input_thread.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = OperatorControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
