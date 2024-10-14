import cv2
import numpy as np
from time import sleep
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images


class Archivador(Node):

    def __init__(self):
        super().__init__('Archivador')                            # /video_frames   simulador: /camera/image_raw
        self.subscription = self.create_subscription(CompressedImage, '/video_frames', self.listener_callback, 10)
        self.subscription

        self.br = CvBridge()

    def listener_callback(self, msg):
        self.get_logger().info('Receiving video frame')

        #cv2.imwrite('Img_sub_sin_br.png', msg.data)
        with open('Img_sub_sin_br.png', 'wb') as archivo:
            archivo.write(msg.data)

        # Convert ROS Image message to OpenCV image
        img = self.br.compressed_imgmsg_to_cv2(msg)

        Iwidth = 640
        Iheight = 480

        # Resize image
        img = cv2.resize(img, (Iwidth, Iheight))
        #_, img = cv2.imencode('.png', img, [cv2.IMWRITE_PNG_COMPRESSION, 95])
        # Save image
        cv2.imwrite('Img_sub.png', img)


def main(args=None):

    rclpy.init(args=args)
    linea_sub = Archivador()
    rclpy.spin(linea_sub)
    linea_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
