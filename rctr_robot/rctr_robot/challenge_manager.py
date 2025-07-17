#!/usr/bin/env python3
"""
Challenge Manager Node
Coordinates complex challenge sequences and provides high-level strategy
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32MultiArray
from geometry_msgs.msg import Twist
import json
import time

class ChallengeManager(Node):
    def __init__(self):
        super().__init__('challenge_manager')
        
        # Track progress
        self.challenges_completed = []
        self.current_position = 'start'
        self.map_knowledge = {}
        
        # Publishers
        self.strategy_pub = self.create_publisher(
            String, 'strategy_command', 10)
        
        # Subscribers
        self.state_sub = self.create_subscription(
            String, 'robot_state', self.state_callback, 10)
        
        # Challenge sequence based on track layout
        self.challenge_sequence = [
            'line_following',
            'door',
            'slope',
            'ditch',
            'qr_code',
            'bump',
            'stairs',
            'obstacle',
            'uneven_terrain',
            'roller',
            'traffic_light',
            'finish'
        ]
        
        # IMU data for localization would go here
        
        self.get_logger().info('Challenge manager initialized')
    
    def state_callback(self, msg):
        """Monitor robot state and provide strategic decisions"""
        try:
            state_data = json.loads(msg.data)
            
            # Log progress
            if state_data.get('current_challenge'):
                challenge = state_data['current_challenge']
                if challenge not in self.challenges_completed:
                    self.challenges_completed.append(challenge)
                    self.get_logger().info(
                        f'Completed: {challenge} '
                        f'({len(self.challenges_completed)}/10)')
            
        except Exception as e:
            self.get_logger().error(f'State parsing error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ChallengeManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
