import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from geometry_msgs.msg import Twist, Vector3
import re


class ImagePublisher(Node):

    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, '/video', 10)
        self.bridge = CvBridge()

        # We will publish a message every 0.001 seconds
        timer_period = 0.001  # seconds

        # Create the timer
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.listener_callback, 10)

    def publish_image(self, img):
        try:
            if img is None:
                return
            ros_image = self.bridge.cv2_to_imgmsg(img)
        except CvBridgeError as e:
            self.get_logger().info('Error in conversion to image message: %s' % e)
            return

        self.publisher_.publish(ros_image)

    def timer_callback(self):
        #br = CvBridge()
        try:
            with open("Img_sub_sin_br.png", 'rb') as archivo:
                msg = archivo.read()

            buf = np.ndarray(shape=(1, len(msg)),
                             dtype=np.uint8, buffer=msg)
            img = cv2.imdecode(buf, cv2.IMREAD_ANYCOLOR)

            self.publish_image(img)
            #self.get_logger().info('Publishing video frame')
        except cv2.error:
            print("Error al leer la imagen")
            # continue

    def listener_callback(self, msg):
        self.get_logger().info('Receiving cmd_vel')
        print(msg)
        with open("cmd_vel", 'w') as archivo:
            archivo.write(str(msg))

        print("prueba_lectura()")
        print(self.prueba_lectura())

    def prueba_lectura(self):
        with open("cmd_vel", 'r') as archivo:
            s = archivo.read()
        return self.str_to_twist(s)

    def str_to_twist(self, s):
        # Extraer los valores de los componentes lineales y angulares
        matches = re.findall(r"[-+]?\d*\.\d+|\d+", s)
        print(matches)

        # Crear un nuevo mensaje Twist con los valores extraídos
        twist = Twist()
        twist.linear.x = float(matches[1])
        twist.linear.y = float(matches[2])
        twist.linear.z = float(matches[3])
        twist.angular.x = float(matches[5])
        twist.angular.y = float(matches[6])
        twist.angular.z = float(matches[7])

        return twist


def main(args=None):
    rclpy.init(args=args)

    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
