import cv2
import numpy as np
from time import sleep
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images


class Linea_sub(Node):

    def __init__(self):
        super().__init__('linea_sub')
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.listener_callback, 10)
        self.subscription

        self.br = CvBridge()

    def listener_callback(self, msg):
        self.get_logger().info('Receiving video frame')

        # Convert ROS Image message to OpenCV image
        img = self.br.imgmsg_to_cv2(msg)

        # Display image
        cv2.imshow("camera", img)
        cv2.waitKey(1)


def main(args=None):

    rclpy.init(args=args)
    linea_sub = Linea_sub()
    rclpy.spin(linea_sub)
    linea_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
