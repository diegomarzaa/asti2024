import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
import pytesseract
from pytesseract import Output
import imutils


class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.br = CvBridge()

        self.caracteres = [input("Ingrese el caracter numero: "), input("Ingrese el caracter letra: ")]

        self.primer = True

    def listener_callback(self, data):
        #self.get_logger().info('Receiving video frame')
        # Convert ROS Image message to OpenCV image
        img = self.br.imgmsg_to_cv2(data)
        hImg, wImg, _ = img.shape

        # img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        try:
            # Usamos Tesseract para extraer la orientación del texto en la imagen.
            results = pytesseract.image_to_osd(img, output_type=Output.DICT)

            # Mostramos la información sobre la orientación.
            print(f'Orientación detectada: {results["orientation"]}')
            print(f'Una rotación de {results["rotate"]}° es necesaria para corregir.')
            print(f'Tipo de escritura detectada: {results["script"]}')

            # Rotamos la imagen
            rotated = imutils.rotate_bound(img, angle=results['rotate'])

            cv2.imshow('Rotada', rotated)

        except:
            rotated = img

        boxes = pytesseract.image_to_boxes(rotated)

        # print(pytesseract.image_to_string(img))

        options = ''
        options += f'-c tessedit_char_whitelist=0123456789qwertyuiopasdfghjklñzxcvbnmQWERTYUIOPASDFGHJKLÑZXCVBNM'
        options += f'-c tessedit_char_blacklist=~.!?@#$%&*()_+'

        cv2.circle(img, (wImg // 2, hImg), 5, (50, 50, 255), 2)

        for b in boxes.splitlines():
            # print(b)
            b = b.split(' ')

            print(b)
            x, y, w, h = int(b[1]), int(b[2]), int(b[3]), int(b[4])
            # if b[0] in caracteres:
            pintar(x, y, w, h, img, b, hImg, wImg)

        # Obtener los datos de la imagen
        data = pytesseract.image_to_data(img, config='outputbase digits', output_type=pytesseract.Output.STRING)

        # Calcular el porcentaje de confianza promedio
        confianza_promedio = data

        #print(confianza_promedio)

        """ El subcritor lo que hara sera agarrar las coordenadas dadas por el publisher para mover
            El motor a la dirreccion que queremos."""

        cv2.imshow("camera", img)

        cv2.waitKey(1)


def pintar(x, y, w, h, img, b, hImg, wImg):
    #print(x, y, w, h, b, hImg, wImg)
    cv2.rectangle(img, (x, hImg - y), (w, hImg - h), (50, 50, 255), 2)
    cv2.line(img, (wImg//2, hImg), (x, hImg - y), (50, 50, 255), 2)
    cv2.putText(img, b[0], (x, hImg - y + 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (50, 50, 255), 2)


def main(args=None):
    pytesseract.pytesseract.tesseract_cmd = "/bin/tesseract"

    rclpy.init(args=args)

    image_subscriber = ImageSubscriber()

    rclpy.spin(image_subscriber)

    image_subscriber.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()