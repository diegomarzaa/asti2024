import cv2
import numpy as np
from time import sleep
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
from sensor_msgs.msg import CompressedImage


class Linea_sub(Node):

    def __init__(self):
        super().__init__('linea_sub')                            # /video_frames   simulador: /camera/image_raw
        self.sim = input("¿Es raspberry? (s/n): ").lower() == 's'
        if self.sim:
            self.subscription = self.create_subscription(CompressedImage, '/video', self.listener_callback, 10)
        else:
            self.subscription = self.create_subscription(CompressedImage, '/video_frames', self.listener_callback, 10)
        self.subscription

        self.br = CvBridge()

    def listener_callback(self, msg):
        #self.get_logger().info('Receiving video frame')

        # Convert ROS Image message to OpenCV image
        if self.sim:
            img = self.br.compressed_imgmsg_to_cv2(msg)
        else:
            img = self.br.compressed_imgmsg_to_cv2(msg)
        #img = cv2.imdecode(np.frombuffer(img, np.uint8), cv2.IMREAD_UNCHANGED)

        # Display image
        if img is not None:
            print("prueba")
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
