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
                                                                      #I imp                #L DIFICL IZQU
        self.chars = ['A,(4)', 'B,8', 'C', 'D', 'E', 'F', 'G', 'H,N', 'I', 'J', 'K,(k, X)', 'L, ( [,( )', 'M,N,H',
                      '1', '2', '3, (8, 9)', '4', '5', '6', '7', '8', '9', '10', '11', '12', '13']
        self.mov = Movements()

        self.posicion = int(input("En que lado de la cuadricula esta el robot? "
                                    "1. Izquierda\n"
                                    "2. Derecha\n"
                                    "3. Arriba\n"
                                    "4. Abajo\n"))

    def listener_callback(self, data):
        #self.get_logger().info('Receiving video frame')
        # Convert ROS Image message to OpenCV image
        img = self.br.imgmsg_to_cv2(data)
        hImg, wImg, _ = img.shape

        #prueba = img.copy()
        # Aplicamos OCR utilizando los lenguajes definidos anteriormente.
        reader = Reader(['en', 'es'], gpu=True)

        if self.posicion == 1:
            img = self.rotar_img(img, 90)
            hImg, wImg, _ = img.shape
        elif self.posicion == 2:
            img = self.rotar_img(img, 270)
            hImg, wImg, _ = img.shape
        elif self.posicion == 3:
            img = self.rotar_img(img, 180)
            hImg, wImg, _ = img.shape

        results = reader.readtext(img)

        encontrado = False
        #print(results)
        if len(results) != 0:
            # Iteramos sobre las predicciones del modelo de EasyOCR.
            for bounding_box, text, probability in results:
                # Imprimimos la probabilidad del texto.
                #print(f'{probability:.4f}: {text}')

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
                cv2.waitKey(1)

                palabra = text.split()
                print(palabra)

                for i in range(len(palabra)):
                    if palabra[i] in self.caracteres:
                        print(f"Detectado {text}")
                        self.girar_hasta_centro((tl, br), wImg, hImg)
                        #self.mov.detener()
                        self.mov.avanzar()
                        time.sleep(1)
                        encontrado = True
                        break
                if text in self.caracteres:
                    break
        if not encontrado:
            self.mov.avanzar()

        # Mostramos el resultado en pantalla.
        cv2.imshow('Resultado', img)
        cv2.waitKey(1)

    def rotar_img(self, img, angle):
        # Rotamos la imagen.
        return imutils.rotate_bound(img, angle)

    def girar_hasta_centro(self, bounding_box, image_width, image_height):
        # 1. Calcular la columna central de la imagen
        if self.posicion == 1 or self.posicion == 2:
            center_column = image_height // 2
        else:
            center_column = image_width // 2

        # 2. Determinar si las coordenadas de la caja delimitadora están a la izquierda o a la derecha de la columna central
        if self.posicion == 1 or self.posicion == 2:
            box_center = (bounding_box[0][1] + bounding_box[1][1]) / 2
        else:
            box_center = (bounding_box[0] + bounding_box[1]) / 2

        if self.posicion == 1:
            if box_center < center_column:
                # 3. Si las coordenadas están a la izquierda, necesitamos girar a la derecha
                print("Girar izquierda")
                self.mov.girar_izquierda()
                time.sleep(0.1)
            elif box_center > center_column:
                # 3. Si las coordenadas están a la derecha, necesitamos girar a la izquierda
                print("Girar derecha")
                self.mov.girar_derecha()
                time.sleep(0.1)

        elif self.posicion == 2:
            if box_center < center_column:
                # 3. Si las coordenadas están a la izquierda, necesitamos girar a la derecha
                print("Girar derecha")
                self.mov.girar_derecha()
                time.sleep(0.1)
            elif box_center > center_column:
                # 3. Si las coordenadas están a la derecha, necesitamos girar a la izquierda
                print("Girar izquierda")
                self.mov.girar_izquierda()
                time.sleep(0.1)

        elif self.posicion == 3:
            if box_center < center_column:
                # 3. Si las coordenadas están a la izquierda, necesitamos girar a la derecha
                print("Girar derecha")
                self.mov.girar_derecha()
                time.sleep(0.1)
            elif box_center > center_column:
                # 3. Si las coordenadas están a la derecha, necesitamos girar a la izquierda
                print("Girar izquierda")
                self.mov.girar_izquierda()
                time.sleep(0.1)

        else:
            if box_center < center_column:
                # 3. Si las coordenadas están a la izquierda, necesitamos girar a la izquierda
                print("Girar izquierda")
                self.mov.girar_izquierda()
                time.sleep(0.1)
            elif box_center > center_column:
                # 3. Si las coordenadas están a la derecha, necesitamos girar a la derecha
                print("Girar derecha")
                self.mov.girar_derecha()
                time.sleep(0.1)


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