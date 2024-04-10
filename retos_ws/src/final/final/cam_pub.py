# Basic ROS 2 program to publish real-time streaming
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com

# Import the necessary libraries
import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
from sensor_msgs.msg import CompressedImage  # CompressedImage is the message type


class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(CompressedImage, '/video_frames', 1)

        # We will publish a message every 0.001 seconds
        timer_period = 0.001  # seconds

        # Create the timer
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.cap = cv2.VideoCapture(0)

        self.br = CvBridge()

    def timer_callback(self):
        ret, img = self.cap.read()

        if ret:
            msg = self.br.cv2_to_compressed_imgmsg(img, dst_format='png')
            self.publisher_.publish(msg)
            with open('Img_sub_sin_br.png', 'wb') as archivo:
                archivo.write(msg.data)

        self.get_logger().info('Publishing video frame')





def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()