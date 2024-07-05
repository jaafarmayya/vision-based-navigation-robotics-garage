import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
import numpy as np
import time

class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.previous_error = 0
        self.integral = 0

    def compute(self, error, delta_time):
        self.integral += error * delta_time
        derivative = (error - self.previous_error) / delta_time
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.previous_error = error
        return output

def process_image(img):
    # Resize the image to 640x480
    resized_img = cv2.resize(img, (640, 480))

    # Rotate the image 90 degrees clockwise
    rotated_img = cv2.rotate(resized_img, cv2.ROTATE_90_CLOCKWISE)

    # Crop the image height to take the upper half
    height, width = rotated_img.shape[:2]
    cropped_img = rotated_img[:height-285, :2*188]

    return cropped_img

# Initialize PID controller
pid = PID(Kp=0.15, Ki=0, Kd=0)

lower1 = np.array([10, 43, 0])
upper1 = np.array([38, 255, 255])
lower_red = np.array([170, 0, 0])
upper_red = np.array([179, 255, 255])

red_threshold = 286 # samsung
x0 = 10
y0 = 5

previous_time = time.time()

class ProcessingNode(Node):
    def __init__(self):
        super().__init__('processing_node')
        self.subscription = self.create_subscription(Image, 'camera_frames', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(String, 'processing_results', 10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        result = self.process_frame(frame)
        self.publisher_.publish(String(data=result))

    def process_frame(self, frame):
        global previous_time  # Declare previous_time as global
        resized_img = process_image(frame)  # Resize to 640x480 or any desired size
        img_height, img_width = resized_img.shape[:2]

        image = cv2.cvtColor(resized_img, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(image, lower1, upper1)
        mask_red = cv2.inRange(image, lower_red, upper_red)
        
        contours1, _ = cv2.findContours(mask1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        pid_output = 0  # Initialize pid_output with a default value

        if contours1:
            # List to hold contours and their centroids
            contours_with_centroids = []
            for contour in contours1:
                if cv2.contourArea(contour) > 100:
                    m = cv2.moments(contour)
                    if m["m00"] != 0:
                        cx = int(m["m10"] / m["m00"])
                        cy = int(m["m01"] / m["m00"])
                        contours_with_centroids.append((contour, (cx, cy)))
        
            # Find the contour with the largest cy value
            if contours_with_centroids:
                largest_contour, (cx, cy) = max(contours_with_centroids, key=lambda x: x[1][1])

                x, y, w, h = cv2.boundingRect(largest_contour)
                cv2.rectangle(resized_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
                
                # Optionally, draw the centroid
                cv2.circle(resized_img, (cx, cy), 5, (255, 0, 0), -1)
                
                # Calculate the error
                error = cx - (img_width // 2)
                current_time = time.time()
                delta_time = current_time - previous_time
                previous_time = current_time

                # Compute the PID output
                pid_output = pid.compute(error, delta_time)
                # send_data(str(round(pid_output)));
                # Receive data from Arduino
                #response = receive_data()
        return str(pid_output)

def main(args=None):
    rclpy.init(args=args)
    node = ProcessingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()