import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int8
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
from pyzbar import pyzbar

# TODO: Add your YOLO-Nano imports and model loading here

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        self.bridge = CvBridge()

        # Subscribers
        self.image_subscriber = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10) # ASSUMPTION: Camera node publishes on /image_raw

        # Publishers
        self.qr_publisher = self.create_publisher(String, '/qr_data', 10)
        self.traffic_light_publisher = self.create_publisher(Int8, '/traffic_light_state', 10)
        self.line_publisher = self.create_publisher(Point, '/line_position', 10)

        # TODO: Load YOLO model here
        # self.yolo_model = ...

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # --- Task-specific processing ---
        self.detect_line(cv_image)
        self.detect_qr_code(cv_image)
        self.detect_traffic_light(cv_image)
        # self.run_yolo_detection(cv_image) # For more complex objects

    def detect_line(self, image):
        # TODO: This is a simplified line detector. Tune thresholds and logic.
        h, w, _ = image.shape
        # Look at the bottom part of the image for the line
        roi = image[h*3//4:, :]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)
        
        M = cv2.moments(thresh)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            
            # Create a Point message
            # x: horizontal error from center, y: confidence, z: unused
            line_pos_msg = Point()
            line_pos_msg.x = float(cx - w / 2)
            line_pos_msg.y = 1.0 # Confidence
            self.line_publisher.publish(line_pos_msg)

    def detect_qr_code(self, image):
        barcodes = pyzbar.decode(image)
        for barcode in barcodes:
            if barcode.type == 'QRCODE':
                qr_data = barcode.data.decode('utf-8')
                self.get_logger().info(f'QR Code detected: {qr_data}')
                msg = String()
                msg.data = qr_data
                self.qr_publisher.publish(msg)

    def detect_traffic_light(self, image):
        # TODO: This is a placeholder. Implement robust color detection.
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # Define HSV ranges for red and green
        red_lower = (0, 120, 70)
        red_upper = (10, 255, 255)
        green_lower = (40, 40, 40)
        green_upper = (80, 255, 255)
        
        mask_red = cv2.inRange(hsv, red_lower, red_upper)
        mask_green = cv2.inRange(hsv, green_lower, green_upper)
        
        red_pixels = cv2.countNonZero(mask_red)
        green_pixels = cv2.countNonZero(mask_green)
        
        msg = Int8()
        if red_pixels > 500: # Threshold pixel count
            msg.data = 0 # 0 for Red
            self.traffic_light_publisher.publish(msg)
        elif green_pixels > 500:
            msg.data = 1 # 1 for Green
            self.traffic_light_publisher.publish(msg)

    # def run_yolo_detection(self, image):
    #     # TODO: Implement YOLO-Nano inference here
    #     # results = self.yolo_model(image)
    #     # Process results and publish detections for objects like the door, stairs, etc.
    #     pass

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
