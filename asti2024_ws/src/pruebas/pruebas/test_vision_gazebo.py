import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('simple_image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Adjust this to match your camera topic
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Display image
            cv2.imshow("Camera Image", cv_image)
            cv2.waitKey(1)  # Refresh display
        except Exception as e:
            self.get_logger().error('Failed to convert image: %r' % (e,))

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    # Clean up OpenCV's window
    cv2.destroyAllWindows()
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()