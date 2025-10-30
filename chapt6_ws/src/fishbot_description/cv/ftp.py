import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class FTP(Node):
    def __init__(self):
        super().__init__('ftp_mission')

        self.bridge = CvBridge()
        # Subscriber: [surge_pwm, lateral_pwm, yaw_pwm]
        self.sub = self.create_subscription(
            Image,
            '/camera_sensor/image_raw',  # expects 3 values: [surge, lateral, yaw]
            self.callback,
            10
        )

        # Publisher: wheel velocities [FL, FR, BL, BR]
        self.pub = self.create_publisher(
            Float64MultiArray,
            'pwm_input',
            10
        )

        self.logger = self.get_logger()
        self.initial = False

    
    def callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow("Camera", cv_image)
        if not self.initial:
            self.logger.info(f"Camera dimension is {cv_image.shape}")
            self.initial = True
        self.logic(cv_image)
        cv2.waitKey(1)

    def logic(self, frame):
        # Red color ranges
        red_lower1 = np.array([0, 100, 100])
        red_upper1 = np.array([10, 255, 255])
        red_lower2 = np.array([160, 100, 100])
        red_upper2 = np.array([179, 255, 255])

        # Detect red
        pos1, mask1 = self.detect_color(frame, red_lower1, red_upper1)
        pos2, mask2 = self.detect_color(frame, red_lower2, red_upper2)
        red_pos = pos1 if pos1 else pos2

        # Move based on red buoy
        self.movement(red_pos)


    def movement(self, red_pos):
        surge = 0
        lateral = 0
        yaw = 0

        if red_pos:
            red_x = red_pos[0]

            # Example: keep red buoy on left side (target x ~100–300)
            if red_x > 500:
                yaw = 200  # turn right
                surge = 1000
            elif red_x > 300:
                yaw = 100
                surge = 1000
            elif red_x <100:
                surge = 800
                yaw = -100
            else:
                surge = 1000
                yaw = 0
        else:
            # Default forward if no buoy detected
            surge = 1000
            yaw = -200  # search left

        msg = Float64MultiArray()
        msg.data = [float(surge), float(lateral), float(yaw)]
        self.pub.publish(msg)

    # Function to detect a color and return its centroid
    def detect_color(self, frame, lower_hsv, upper_hsv):
        
        # Convert frame to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Threshold for the color
        mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # Take the largest contour
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                return (cx, cy), mask
        return None, mask

            


if __name__=="__main__":
    rclpy.init()
    node = FTP()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()