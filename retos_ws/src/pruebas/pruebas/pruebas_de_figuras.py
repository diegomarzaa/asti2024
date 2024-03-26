import cv2
import imutils
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images


class Linea_sub(Node):

    def __init__(self):
        super().__init__('linea_sub')                           # video_frames  /camera/image_raw
        self.subscription = self.create_subscription(Image, 'video_frames', self.listener_callback, 10)
        self.subscription

        self.br = CvBridge()

        self.tonos = {
            "Oscuros": [30, 40],
            "Gama de blancos": [40, 80],
                       #0    1    2    3    4    5    6   7     8    9   10   11   12   13
            "Tibios": [110, 120, 130, 140, 150, 160, 170, 180, 190, 200, 210, 220, 230, 240,
                       #14   15   16   17   18   19   20   21   22   23   24   25   26   27
                       115, 125, 135, 145, 155, 165, 175, 185, 195, 205, 215, 225, 235, 245],
            "Claros": [250, 254],
            "Pruebas": [100.5]
        }

        self.formas = {
            "TRIANGULO": 0,
            "CUADRADO": 0,
            "ARCO": 0,
            "CILINDRO": 0,
            "CIRCULO": 0,
            "ESTRELLA": 0
        }

        self.tiempo = 0.0

    def listener_callback(self, msg):
        #self.get_logger().info('Receiving video frame')

        # Convert ROS Image message to OpenCV image
        img = self.br.imgmsg_to_cv2(msg)

        self.analizar(img)

        self.tiempo += 0.1

        if self.tiempo >= 10.0:
            maximo = max(self.formas, key=self.formas.get)
            print("\n\n\n\n\n\n\n\n\n", maximo, "\n\n\n\n\n\n\n\n\n")
            self.tiempo = 0.0
            self.formas = {
                "TRIANGULO": 0,
                "CUADRADO": 0,
                "ARCO": 0,
                "CILINDRO": 0,
                "CIRCULO": 0,
                "ESTRELLA": 0
            }

        # Display image
        cv2.imshow("camera", img)
        cv2.waitKey(1)

    def detect(self, contour):
        """
        Función que, dado un contorno, retorna la forma geométrica más cercana con base al número de lados del perímetro del
        mismo.
        :param contour: Contorno del que inferiremos una figura geométrica.
        :return: Texto correspondiente a la figura geométrica identificada (TRIANGULO, CUADRADO, RECTANGULO, PENTAGONO o CIRCULO)
        """
        # Hallamos el perímetro (cerrado) del contorno.
        perimeter = cv2.arcLength(contour, True)

        # Aproximamos un polígono al contorno, con base a su perímetro.
        approximate = cv2.approxPolyDP(contour, .03 * perimeter, True)

        print(len(approximate))

        # Si el polígono aproximado tiene 3 lados, entonces es un triángulo.
        if len(approximate) == 3:
            shape = 'TRIANGULO'
            self.formas["TRIANGULO"] += 1
        # Si el polígono aproximado tiene 4 lados, entonces puede ser o un cuadrado o un rectángulo.
        elif len(approximate) == 4:
            # Calculamos la relación de aspecto.
            x, y, w, h = cv2.boundingRect(approximate)
            aspect_ratio = w / float(h)

            # La figura será un cuadrado si la relación de aspecto está entre 95% y 105%, es decir, si todos los lados miden
            # más o menos lo mismo. En caso contrario, se trata de un rectángulo.
            if .95 <= aspect_ratio <= 1.05:
                shape = 'CUADRADO'
                self.formas["CUADRADO"] += 1
            else:
                shape = 'CUADRADO'
                self.formas["CUADRADO"] += 1
        # Si el polígono aproximado tiene 5 lados, es un pentágono.
        elif len(approximate) == 5:
            shape = 'ARCO'
            self.formas["ARCO"] += 1

        # Por defecto, asumiremos que cualquier polígono con 6 o más lados es un círculo.
        elif len(approximate) >= 9 and len(approximate) <= 10:
            shape = 'estrella'
            self.formas["ESTRELLA"] += 1
        elif 7 <= len(approximate) <= 8:
            shape = 'circulo'
            self.formas["CIRCULO"] += 1
        else:
            shape = 'CILINDRO'
            self.formas["CILINDRO"] += 1

        return shape

    def analizar(self, image):

        imagen = [
            "/home/jcrex/uji/teams-robotics/astic/Detector_caracteres/dynamixel.png",
            "/home/jcrex/Imágenes/figuras_geométricas.png",
            "/home/jcrex/Imágenes/dataset_estrellas/estrella.png"
            "/home/jcrex/Imágenes/dataset_estrellas/estrella_3.jpeg",
            "/home/jcrex/Imágenes/dataset_estrellas/estrella_5.png",
            "/home/jcrex/Imágenes/dataset_estrellas/estrella_6.jpeg",
            "/home/jcrex/Imágenes/dataset_estrellas/estrella_7.jpeg",
            "/home/jcrex/Imágenes/dataset_estrellas/estrella_8.jpeg",
            "/home/jcrex/Imágenes/dataset_estrellas/estrella_9.jpeg",
            "/home/jcrex/Imágenes/dataset_estrellas/estrella_4.jpeg",
            "/home/jcrex/Imágenes/dataset_estrellas/estrella_10.jpeg",
            "/home/jcrex/Imágenes/circulo_rojo.jpg",
        ]
        #image = cv2.imread(imagen[10])

        # Cargamos la imagen de entrada y la redimensionamos.
        resized = imutils.resize(image, width=380)

        # Calculamos la relación de proporción entre la imagen original y la redimensionada.
        ratio = image.shape[0] / float(resized.shape[0])


        # Convertimos la imagen a escala de grises, la difuminamos y la binarizamos.
        gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (3, 3), 0)   # Mejor resultado con (1, 1), 1

        cv2.imshow("blurred", blurred)
        cv2.waitKey(1)

        # =============================================================================
        # =============================================================================
        # =============================================================================

        thresholded = cv2.threshold(blurred, self.tonos["Tibios"][0], 255, cv2.THRESH_BINARY)[1]
        cv2.imshow("thresholded", thresholded)
        cv2.waitKey(1)

        # Invierte los colores de la imagen
        inverted_image = cv2.bitwise_not(thresholded)
        cv2.imshow("Inverted Image", inverted_image)
        cv2.waitKey(1)

        thresholded = inverted_image

        # =============================================================================
        # =============================================================================
        # =============================================================================

        # Hallamos los contornos.
        contours = cv2.findContours(thresholded.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)

        # Iteramos sobre cada contorno...
        for contour in contours:
            # Calculamos los momentos del contorno para encontrar su centro.
            M = cv2.moments(contour)
            #print(M, contour)
            try:
                center_x = int((M['m10'] / M['m00']) * ratio)
                center_y = int((M['m01'] / M['m00']) * ratio)
            except ZeroDivisionError:
                continue

            # Determinamos la forma geométrica del contorno usando la función que definimos previamente.
            shape = self.detect(contour)

            # Ajustamos el contorno a la imagen original.
            contour = contour.astype('float') * ratio
            contour = contour.astype('int')

            # Dibujamos el contorno y lo etiquetamos con el nombre de la figura geométrica identificada.
            cv2.drawContours(image, [contour], -1, (0, 255, 0), 2)
            cv2.putText(image, shape, (center_x, center_y), cv2.FONT_HERSHEY_SIMPLEX, .7, (255, 0, 0), 2)

            # Mostramos el contorno en la imagen original.
            #cv2.imshow('Imagen', image)
            #cv2.waitKey(0)


def main(args=None):

    rclpy.init(args=args)
    linea_sub = Linea_sub()
    rclpy.spin(linea_sub)
    linea_sub.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
