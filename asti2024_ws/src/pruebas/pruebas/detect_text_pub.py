import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
from std_msgs.msg import Int32MultiArray, MultiArrayDimension
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
from Model_detect_text import CamAstic
import pytesseract
import numpy as np


class ImagePublisher(Node):

    def __init__(self):

        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, 'video_frames', 10)
        self.array_publisher_ = self.create_publisher(Int32MultiArray, 'array_topic', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.cap = cv2.VideoCapture(0)
        self.br = CvBridge()

        self.cap.set(3, 640)
        self.cap.set(4, 480)

    def timer_callback(self):
        ret, img = self.cap.read()
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        cv2.circle(img, (320, 480), 5, (50, 50, 255), 2)
        hImg, wImg, _ = img.shape
        boxes = pytesseract.image_to_boxes(img)

        for b in boxes.splitlines():
            print(b)
            b = b.split(' ')
            # print(b)
            x, y, w, h = int(b[1]), int(b[2]), int(b[3]), int(b[4])
            cv2.rectangle(img, (x, hImg - y), (w, hImg - h), (50, 50, 255), 2)
            cv2.line(img, (320, 480), (x, hImg - y), (50, 50, 255), 2)
            cv2.putText(img, b[0], (x, hImg - y + 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (50, 50, 255), 2)

            if ret == True:
                array_msg = Int32MultiArray()
                array_msg.data = [x, y, w, h]
                self.array_publisher_.publish(array_msg)

        cv2.imshow("Result", img)
        self.publisher_.publish(self.br.cv2_to_imgmsg(img))
        self.get_logger().info('Publishing video frame')


def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()