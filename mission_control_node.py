import rclpy
from rclpy.node import Node
from enum import Enum
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import String, Int8, Float32MultiArray
from nav_msgs.msg import Odometry

class RobotState(Enum):
    IDLE = 0
    LINE_FOLLOWING = 1
    QR_TASK = 2
    TRAFFIC_LIGHT_WAIT = 3
    OBSTACLE_AVOIDANCE = 4
    STAIR_CLIMB = 5
    # TODO: Add more states for each challenge task

class MissionControlNode(Node):
    def __init__(self):
        super().__init__('mission_control_node')
        self.state = RobotState.LINE_FOLLOWING # Initial state

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_servo_publisher = self.create_publisher(Float32MultiArray, '/cmd_servo', 10)

        # Subscribers
        self.line_subscriber = self.create_subscription(
            Point, '/line_position', self.line_callback, 10)
        self.qr_subscriber = self.create_subscription(
            String, '/qr_data', self.qr_callback, 10)
        self.traffic_light_subscriber = self.create_subscription(
            Int8, '/traffic_light_state', self.traffic_light_callback, 10)
        self.odom_subscriber = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # State machine timer
        self.timer = self.create_timer(0.1, self.state_machine_update) # 10 Hz

        # State variables
        self.line_error = 0.0
        self.last_qr_data = None
        self.traffic_light_status = -1 # -1: unknown, 0: red, 1: green

    def line_callback(self, msg):
        self.line_error = msg.x

    def qr_callback(self, msg):
        self.last_qr_data = msg.data
        if self.state == RobotState.LINE_FOLLOWING:
            self.get_logger().info("QR Code detected, switching to QR_TASK state.")
            self.state = RobotState.QR_TASK

    def traffic_light_callback(self, msg):
        self.traffic_light_status = msg.data

    def odom_callback(self, msg):
        # Can be used for tracking distance traveled to trigger state changes
        pass
        
    def state_machine_update(self):
        """Main logic loop"""
        if self.state == RobotState.IDLE:
            self.stop_robot()
        
        elif self.state == RobotState.LINE_FOLLOWING:
            self.follow_line()

        elif self.state == RobotState.QR_TASK:
            self.handle_qr_task()
            # After handling, decide next state
            # For now, just go back to line following after a delay
            # TODO: Implement a more robust transition
            self.state = RobotState.LINE_FOLLOWING 

        # Add more state handlers here...
        # elif self.state == RobotState.STAIR_CLIMB:
        #     self.climb_stairs()

    def follow_line(self):
        twist_msg = Twist()
        # TODO: Tune these P-controller values
        KP_ANGULAR = 0.004
        MAX_ANGULAR_SPEED = 0.5
        BASE_LINEAR_SPEED = 0.15

        twist_msg.linear.x = BASE_LINEAR_SPEED
        angular_z = -KP_ANGULAR * self.line_error
        twist_msg.angular.z = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, angular_z))
        
        self.cmd_vel_publisher.publish(twist_msg)

    def handle_qr_task(self):
        self.stop_robot()
        self.get_logger().info(f'Handling QR Task with data: {self.last_qr_data}')
        # TODO: Implement logic to change on-board LED based on qr_data
        # This might involve publishing to another topic that the ESP32 subscribes to.

    def stop_robot(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist_msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = MissionControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
