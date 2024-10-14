import time

import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
from easyocr import Reader
import imutils

from final.Movements import Movements


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

        self.defrente = False

        self.chars = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M',
                      '1', '2', '3', '4', '5', '6', '7', '8', '9', '10', '11', '12', '13']
        self.mov = Movements()

    def listener_callback(self, data):
        #self.get_logger().info('Receiving video frame')
        # Convert ROS Image message to OpenCV image
        img = self.br.imgmsg_to_cv2(data)
        hImg, wImg, _ = img.shape

        #prueba = img.copy()
        # Aplicamos OCR utilizando los lenguajes definidos anteriormente.
        reader = Reader(['en', 'es'], gpu=True)
        results = reader.readtext(img)
        if len(results) != 0:
            # Iteramos sobre las predicciones del modelo de EasyOCR.
            for bounding_box, text, probability in results:
                # Imprimimos la probabilidad del texto.
                print(f'{probability:.4f}: {text}')

                # Extraemos y ajustamos las coordenadas de la detección.
                tl, tr, br, bl = bounding_box
                tl = (int(tl[0]), int(tl[1]))
                tr = (int(tr[0]), int(tr[1]))
                br = (int(br[0]), int(br[1]))
                bl = (int(bl[0]), int(bl[1]))

                # Limpiamos el texto, y lo mostramos en la imagen.
                text = cleanup_text(text)
                cv2.rectangle(img, tl, br, (0, 255, 0), 2)
                cv2.putText(img, text, (tl[0], tl[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, .8, (0, 255, 0), 2)

            # Mostramos el resultado en pantalla.
            cv2.imshow('Resultado', img)
            if text in self.chars:
                self.mov.detener()
                self.mov.avanzar()
                time.sleep(10)
            else:
                self.girar_hasta_centro((tl[0], tl[1], br[0], br[1]), wImg)

            cv2.waitKey(1)

    def rotar_img(self, img, angle):
        # Rotamos la imagen.
        return imutils.rotate_bound(img, angle)

    def girar_hasta_centro(self, bounding_box, image_width):
        # 1. Calcular la columna central de la imagen
        center_column = image_width / 2

        # 2. Determinar si las coordenadas de la caja delimitadora están a la izquierda o a la derecha de la columna central
        box_center = (bounding_box[0] + bounding_box[2]) / 2

        if box_center < center_column:
            # 3. Si las coordenadas están a la izquierda, necesitamos girar a la derecha
            direction = 'der'
        else:
            # 3. Si las coordenadas están a la derecha, necesitamos girar a la izquierda
            direction = 'izq'

        # 4. Calcular los grados que el robot necesita girar
        # Esto es solo un ejemplo, puedes necesitar ajustar la fórmula para que se ajuste a tus necesidades
        degrees = abs(box_center - center_column) / image_width * 90

        # 5. Llamar a la función `movements.girar_grados(degrees, direccion)` con los grados y la dirección calculados

        self.mov.girar_grados(degrees, direction)


def cleanup_text(text):
    return ''.join([c if ord(c) < 128 else '' for c in text]).strip()


def pintar(x, y, w, h, img, b, hImg, wImg):
    #print(x, y, w, h, b, hImg, wImg)
    cv2.rectangle(img, (x, hImg - y), (w, hImg - h), (50, 50, 255), 2)
    cv2.line(img, (wImg//2, hImg), (x, hImg - y), (50, 50, 255), 2)
    cv2.putText(img, b[0], (x, hImg - y + 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (50, 50, 255), 2)


def main(args=None):
    rclpy.init(args=args)

    image_subscriber = ImageSubscriber()

    rclpy.spin(image_subscriber)

    image_subscriber.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()